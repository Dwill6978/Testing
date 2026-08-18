# Understanding `gui_GliderControlTest.py` — a guided tour

*Written for someone with a CS background who hasn't written serious code in a
while. I assume you remember what a variable, a function, a class, and a loop
are, but that terms like "context manager," "the GIL," or "signal/slot" are
fuzzy. We'll rebuild that vocabulary as we go, always tying it back to a real
line in your file.*

Read this next to the source. I cite line numbers like `(L675)` so you can jump.
(Line numbers track the current file; if you edit the code they'll drift.)

---

## 0. The one-paragraph mental model

This program does three things at once: it **talks to a flying robot over a
radio**, it **draws live graphs**, and it **listens to your mouse/keyboard/game
controller**. Those three jobs have wildly different timing needs, so the program
splits them across **threads** (independent streams of execution) and connects
those threads with a few carefully chosen **thread-safe hand-off mechanisms**.
Almost every design decision in this file exists to answer one question: *"which
thread is allowed to touch this piece of data, and how does data cross safely
from one thread to another?"* Hold onto that question — it's the spine of the
whole program.

---

## 1. Why threads at all? (the core theory)

A **thread** is a sequence of instructions the CPU runs. A normal Python script
has exactly one thread: line runs after line, top to bottom. That's fine until
you need to do two things that *both* want to "wait" or "loop forever":

- The **GUI** must run an infinite loop that watches for clicks and repaints the
  screen. If that loop ever stops, the window freezes ("Not Responding").
- The **control loop** (L675, `_control_loop`) must run its own tight `while`
  loop ~100 times per second, sending commands to the aircraft and reading
  telemetry.

You cannot run two infinite loops in one thread — the first one never returns.
So the program uses **two threads**:

1. The **GUI thread** (also called the "main thread") — owns every widget and the
   matplotlib canvas.
2. The **worker thread** — owns the radio connection and the control loop.

Started at L432:
```python
self._thread = threading.Thread(target=self._run, daemon=True)
self._thread.start()
```
`target=self._run` says "when this thread starts, call `self._run`." `.start()`
launches it running *alongside* the caller — control returns immediately to the
GUI while `_run` executes in parallel.

`daemon=True` means "if the whole program exits, don't wait for this thread —
just kill it." A non-daemon thread would keep Python alive after you close the
window. (There's still a clean shutdown path at L1514, `closeEvent`, which
politely asks the thread to stop and `join`s it — waits for it to finish — with a
timeout so a hung radio can't hang the app forever.)

### 1a. The catch: the GIL and why locks still matter

You may have heard Python has a **Global Interpreter Lock (GIL)** — only one
thread runs Python bytecode at a time. So why bother with threads, and why do we
still need locks?

- **Why threads help anyway:** when the worker thread is *waiting* on the radio
  (I/O), the GIL is released and the GUI thread runs. Threads are perfect for
  "waiting on the outside world" work, which is exactly what radio + GUI are.
- **Why locks are still needed:** the GIL protects a *single* bytecode
  instruction, but not a *sequence* of them. `self.rssi.append(x)` is one safe
  op, but "read a list, then read another list, then combine them" can be
  interrupted halfway. If two threads touch the same data and at least one
  writes, you need a **lock** to make a group of operations *atomic*
  (all-or-nothing). More on this in §3.

---

## 2. The file's skeleton (top-to-bottom map)

Skim these so you know the neighborhoods before we zoom in:

| Lines | What lives there | Role |
|-------|------------------|------|
| 1–24  | Module docstring | Human-readable overview |
| 26–67 | Imports | Pull in tools; note the *try/except* fallbacks |
| 73–108 | Constants | Named magic numbers (incl. `TRIM_PARAMS`, `NOTES_FILE`) |
| 111–131| `dataclass`es + `clamp` | Plain data holders |
| 133–211 | `CsvLogBundle` | Writes telemetry (and the console mirror) to disk |
| 214–279 | `PlotBuffers` | **Thread-safe** data hand-off |
| 282–350 | `PlotCanvas` | The embedded matplotlib graphs |
| 353–396 | `SessionConfig`, `LiveControl` | Two kinds of shared state |
| 398–466 | `ControllerProfile`, `XBOX_PROFILE`, `RC_PROFILE` | Per-controller data profiles (see §7f) |
| 468–1173 | `GliderWorker` | The worker thread: radio + control loop |
| 1175–1191 | `BUTTON_MAPPING` | Static reference data |
| 1193–1716 | `MainWindow` | Every widget + GUI-side logic |
| 1718–1726 | `main()` + `__main__` guard | Program entry point |

Notice the ordering: **things are defined before they're used**. Python reads
top-to-bottom, so a class must exist before another class references it. The one
that runs *everything*, `main()`, is at the very bottom — because it needs all
the others to already be defined.

---

## 3. Data handling: how information moves without corrupting itself

This is the most important section. Three different data-transfer problems, three
different tools. Learning *why each tool fits its problem* is the real lesson.

### 3a. Continuous streaming data → `deque` + `Lock` (`PlotBuffers`, L203)

Telemetry pours in from the aircraft (gyro rates, motor commands, battery). The
graphs only ever need the *recent* history — a rolling window. The perfect data
structure is a **`deque`** ("deck," double-ended queue) created with a `maxlen`:

```python
d = lambda stream: deque(maxlen=self._maxlen(stream))   # L223
```
A `deque(maxlen=N)` automatically drops the oldest item when you append the
(N+1)th. So the buffer can never grow unbounded — memory stays flat no matter how
long you fly. That's a classic **ring buffer**, and `deque` gives it to you for
free. (`.append()` and the auto-eviction are also individually thread-safe, which
is a nice bonus, though we still lock — see below.)

`self._maxlen` (L218) computes the window size from *time*: if you want 20 s of
history and samples arrive every 10 ms, you need `20 / 0.010 = 2000` slots. This
is why every graph shows the *same time span* even though different streams log
at different rates — each deque is sized for its own rate.

**The lock.** Look at every method that touches the deques:
```python
def add_connection(self, ts, rssi):
    with self._lock:                     # L249
        self.t_conn.append(ts); self.rssi.append(rssi)
```
`self._lock = threading.Lock()` (L213). `with self._lock:` is a **context
manager** (the `with` statement): it acquires the lock on entry and *guarantees*
release on exit, even if an exception is thrown inside. This is the same `with`
pattern you've seen for files (`with open(...)`), and it's the idiomatic way to
handle any "acquire → do work → release" resource in Python.

Why lock here? The **producer** is a cflib callback running on cflib's own thread
(L613 `_on_controller_log` etc.), and the **consumer** is the GUI redraw timer
(L312 `refresh`). One writes while the other reads. Without the lock, the reader
could catch the writer mid-update and see half-written state, or the two
appends could interleave. This producer/consumer split across threads is a
textbook concurrency pattern, and the lock is what makes it safe.

**The snapshot trick (L258).** `refresh` doesn't hold the lock while it draws
(drawing is slow; holding a lock that long would stall the producers). Instead it
grabs a *copy* under the lock and releases immediately:
```python
def snapshot(self):
    with self._lock:
        return {name: list(getattr(self, name)) for name in (...)}
```
`list(some_deque)` makes a fresh, independent list. Now the GUI can take its time
plotting the copy while new telemetry flows into the real deques untouched. This
"**copy under lock, then work on the copy lock-free**" pattern is worth
memorizing — it minimizes how long any lock is held, which is the key to keeping
concurrent code fast *and* correct.

`getattr(self, name)` (L260) is **reflection**: fetching an attribute *by its
string name* at runtime. It lets one line handle 17 buffers instead of 17 lines.
The `{k: v for ... in ...}` is a **dict comprehension** — same idea as a list
comprehension but building a dictionary.

### 3b. One-shot commands GUI → worker → `queue.Queue` (L402, L438)

When you click "Arm Motor," the GUI thread must tell the worker thread to do
something *once*. You can't just call the worker's method directly — that would
run the arming code *on the GUI thread*, which doesn't own the radio. Instead the
GUI drops a message in a **thread-safe queue**:

```python
def post(self, action, payload=None):     # L438
    self._cmd_queue.put((action, payload))
```
`queue.Queue` is built for exactly this: multiple threads can `put`/`get` without
you writing any locks — the locking is inside. The worker drains it at the top of
every loop iteration:
```python
def _drain_commands(self):                # L725
    while True:
        try:
            action, payload = self._cmd_queue.get_nowait()
        except queue.Empty:
            return
        self._handle_command(action, payload)
```
`get_nowait()` returns instantly; if the queue is empty it raises `queue.Empty`,
which we catch to break out. The `(action, payload)` **tuple** is a tiny message
format — a string naming the action plus optional data. `_handle_command` (L733)
is one big `if/elif` chain that turns those strings back into radio calls. This is
the **command pattern**: actions are captured as data ("arm", payload) and
executed later, on the correct thread.

**Why a queue for commands but a lock for telemetry?** Commands are discrete
events that must each happen exactly once, in order — a queue preserves order and
delivers each item once. Telemetry is a continuously overwritten "latest value" —
a lock-guarded buffer fits that. Matching the tool to the data's *shape* is the
skill here.

### 3c. Continuous control values GUI → worker → lock-guarded object (`LiveControl`, L366)

Some values aren't one-shot events *or* streams — they're "the current setting,"
like throttle position or the override slider values. The GUI updates them
whenever you move a widget; the worker reads the latest value every loop. That's
the `LiveControl` dataclass, guarded by the worker's lock:

```python
def update_live(self, **kwargs):          # L442
    with self._lock:
        for k, v in kwargs.items():
            setattr(self.live, k, v)
```
`**kwargs` ("keyword arguments") lets callers write
`update_live(throttle=0.5, motor_armed=True)` and receive them as a dict. The
loop `setattr`s each onto the shared object under the lock. The worker reads a
consistent copy with `_live_snapshot` (L448):
```python
def _live_snapshot(self):
    with self._lock:
        return LiveControl(**vars(self.live))
```
`vars(obj)` returns the object's attributes as a dict; `LiveControl(**that)`
rebuilds a fresh independent copy. Same "copy under lock" trick as the plot
buffers — the control loop reads one coherent snapshot per tick, so a value can't
change halfway through the loop's logic.

> **Recap of the data-handling toolbox:**
> - Rolling stream of samples → `deque(maxlen)` + `Lock`, hand off by snapshot.
> - Discrete "do this once" events → `queue.Queue`.
> - "Current setting" values → a lock-guarded shared object.
>
> Three problems, three tools, all solving "cross a thread boundary safely."

### 3d. State that must outlive the program → a plain file (Notes tab, L1324)

Everything above crosses a *thread* boundary but lives and dies with the process:
close the app and it's gone. The Notes tab you added crosses a different
boundary — **time / separate launches**. The tool for that is the oldest one in
the book: read a file on startup, write it back on exit.

```python
NOTES_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "glider_notes.txt")  # L91
```
`__file__` is the path to *this script*; `os.path.dirname(os.path.abspath(...))`
turns it into "the folder this script lives in," so the notes file sits next to
the code no matter what directory you launch from. Building paths with
`os.path.join` instead of gluing strings with `"/"` is the portable habit — it
uses the right separator per OS and avoids double-slash bugs.

The load/save pair is deliberately tiny:
```python
def _load_notes(self):                      # L1350
    try:
        with open(NOTES_FILE, "r", encoding="utf-8") as fh:
            self.notes_edit.setPlainText(fh.read())
    except FileNotFoundError:
        pass                                # first-ever launch: no file yet
    except OSError as exc:
        self._append_console(f"[notes] could not load ...: {exc}\n")
```
Two lessons here. First, **`FileNotFoundError` is expected, not exceptional** —
the very first run has no file, so we catch it and quietly move on rather than
letting it crash the window. Catching a *specific* exception (not a blanket
`except:`) is what lets us treat "missing file" and "disk error" differently.
Second, `encoding="utf-8"` is stated explicitly so the file reads and writes the
same way on every machine instead of guessing a platform default.

`_save_notes` (L1361) is the mirror image — open for writing, dump the whole
buffer. It's called from **two** places: the "Save notes now" button (a manual
flush, in case the app crashes) and `closeEvent` (L1514), so a clean exit always
persists. Loading whole/saving whole means old notes reappear *and* new ones
accumulate, with no risk of duplicating text — simpler and safer than trying to
append just the delta.

> **Fourth tool, different axis:** the first three cross *threads within one
> run*; a file crosses *runs across time*. Same mindset — "who owns this data and
> how does it get somewhere else safely" — pointed at the disk instead of another
> thread.

---

## 4. The `dataclass`es and other Python syntax refreshers

### 4a. `@dataclass` (L100, L108, L342, L366)

```python
@dataclass
class LiveControl:
    trimmed: bool = False
    motor_armed: bool = False
    throttle: float = 0.0
```
The `@dataclass` **decorator** auto-writes the boilerplate `__init__`, `__repr__`,
and `__eq__` for a class that's basically a bag of named fields. Without it you'd
hand-write `def __init__(self, trimmed=False, ...): self.trimmed = trimmed; ...`.
A decorator is just a function that takes a class (or function) and returns a
modified version — `@dataclass` is sugar for `LiveControl = dataclass(LiveControl)`.

The `field: type = default` lines use **type hints** (`bool`, `float`). Python
does **not** enforce them at runtime — they're documentation for humans and tools
(editors, linters). `throttle: float = 0.0` still lets you assign a string; the
hint just says "this is *meant* to be a float."

### 4b. `field(default_factory=...)` — the mutable-default trap (L110)

```python
pitch: PidAxisGains = field(default_factory=lambda: PidAxisGains(...))
```
Why not just `pitch: PidAxisGains = PidAxisGains(...)`? Because of a famous Python
gotcha: a default value is evaluated **once**, when the class is defined, and
*shared by every instance*. If the default were a single mutable object, all your
`PidGains()` would secretly share the same `pitch` object — change one, change
all. `default_factory` says "call this function to make a *fresh* default each
time an instance is created." Rule of thumb: **never use a mutable object
(`list`, `dict`, custom object) as a direct default; use `default_factory`.**

### 4c. `lambda` (L110, L223)

`lambda x: expr` is an anonymous one-line function. `lambda: PidAxisGains(...)`
takes no args and returns a new object. It's used where a throwaway function is
needed inline — as a factory here, and all over the GUI for button handlers
(L1143: `lambda: self.worker.post("arm")`).

### 4d. Generator expressions (L224)

```python
self.t_ctrl, self.gyroroll, self.gyropitch, self.gyroyaw = (d("controller") for _ in range(4))
```
`(expr for x in iterable)` is a **generator** — like a list comprehension but it
produces items lazily, one at a time, instead of building a whole list. Here it
yields four fresh deques, which are unpacked into four variables. The `_` is the
conventional name for "a loop variable I don't care about."

### 4e. `Optional[...]` and `Tuple[...]` (L35)

`Optional[LogConfig]` means "a `LogConfig` **or** `None`." It documents that a
variable is legitimately allowed to be empty (e.g. `self.cf = None` before we
connect). `Tuple[int, int]` means "a 2-tuple of ints." These come from the
`typing` module and, again, are documentation, not enforcement.

### 4f. The import fallbacks (L40, L64) — defensive design

```python
try:
    from PySide6 import QtCore, QtGui, QtWidgets
    ...
except ImportError:
    from PyQt5 import QtCore, ...
```
There are two competing Qt bindings (PySide6, PyQt5). Rather than demand one, the
code *tries* the preferred one and *falls back* to the other. The name `Signal`
differs between them (`pyqtSignal` in PyQt5), so it's aliased so the rest of the
file can just say `Signal`. Same idea at L64 for pygame: if it's not installed,
`pygame = None`, and later code checks `if pygame is None`. This is **graceful
degradation** — the program still runs (minus the controller) instead of crashing
on an import.

---

## 5. The GUI layer (Qt): event loops, signals, and slots

### 5a. The event loop and `main()` (L1522)

```python
def main():
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec() if hasattr(app, "exec") else app.exec_())
```
`app.exec()` **is** the infinite GUI loop mentioned in §1 — it blocks here,
processing clicks, key presses, timers, and repaints until the window closes.
Everything the GUI does happens as a reaction *inside* this loop. `sys.exit(...)`
passes the loop's return code to the OS. The `hasattr(app, "exec")` check is
another binding-compatibility shim (PyQt5 historically spelled it `exec_`).

### 5b. `if __name__ == "__main__":` (L1529)

This guard means "only run `main()` if this file was executed directly, not if it
was `import`ed by another file." `__name__` is `"__main__"` when you run
`python gui_GliderControlTest.py`, but becomes the module's name when imported.
It's the standard way to make a file usable both as a program and as a library.

### 5c. Widgets, layouts, and tabs (`MainWindow`, L1014)

`MainWindow` **inherits** from `QtWidgets.QMainWindow` (`class MainWindow(QMainWindow)`).
`super().__init__()` (L1016) calls the parent's constructor — you must do this so
Qt sets up its internal machinery before you add your own widgets.

The UI is built from **widgets** (buttons, sliders, labels) arranged by
**layouts**. A layout is an invisible manager that positions and resizes its
children automatically. You'll see:
- `QVBoxLayout` — stacks children vertically.
- `QHBoxLayout` — in a row (L1139).
- `QFormLayout` — label/field pairs (L1051).
- `QGridLayout` — a table of rows/columns (L1091, L1177).

Each `_build_*_tab` method (L1049, L1127, L1134, L1234, L1296, L1312, L1324)
constructs one tab's widget tree and returns the top widget, which is added to
the `QTabWidget` (L1029). There are now seven tabs — Setup, Live Plots, Control,
Manual Override, Mapping, Console, and Notes. Splitting UI construction into one method per tab keeps
`__init__` readable — a good **separation-of-concerns** habit.

### 5d. Signals and slots — Qt's event system (and its threading superpower)

Qt objects emit **signals** ("something happened") that connect to **slots**
(functions to run in response). You wire them with `.connect`:
```python
self.connect_btn.clicked.connect(self._toggle_connection)   # L1112
self.throttle_slider.valueChanged.connect(self._on_throttle_changed)  # L1156
```
"When the button is clicked, call `_toggle_connection`." This is the **observer
pattern**: the button doesn't know who's listening; anyone can subscribe.

The truly important part: at the top of `GliderWorker` (L389) *custom* signals
are declared:
```python
console = Signal(str)
telemetry = Signal(float, float)
override_state = Signal(int, int, int, int, int)
trim_value = Signal(str, int)   # (axis, value) trim read back from the deck
```
The worker thread **emits** these (e.g. `self.telemetry.emit(vbat, rssi)`), and
the GUI connects them to its own methods (L1022–1027). **Why this matters:** it is
*illegal* to touch a Qt widget from any thread but the GUI thread. If the worker
tried to call `self.console_view.insertPlainText(...)` directly, you'd get random
crashes. A signal emitted on the worker thread is delivered *on the GUI thread's
event loop*, safely. So signals are the fourth thread-crossing mechanism in this
program — the **worker → GUI** direction — complementing the queue and locks that
go **GUI → worker**. Every arrow between the threads is deliberate:

```
GUI thread  --post(queue)-->        worker thread   (one-shot commands)
GUI thread  --update_live(lock)-->  worker thread   (current settings)
worker/cflib --Lock--> PlotBuffers --Lock--> GUI    (telemetry stream)
worker thread --Signal.emit--> GUI thread          (console/status/slider echo)
```

### 5e. Your new `override_state` echo, as a case study (L1282)

You just added this, so it's a perfect concrete example. The worker computes live
motor commands from the game controller and emits them; the GUI reflects them on
the sliders:
```python
def _on_override_state(self, m1, m2, m3, m4, servo):
    for name, value in (...):
        slider = self.override_sliders[name]
        slider.blockSignals(True)      # <-- the subtle, important line
        slider.setValue(int(value))
        slider.blockSignals(False)
        self.override_value_labels[name].setText(str(int(value)))
```
Why `blockSignals(True)`? Setting a slider's value normally **emits**
`valueChanged`, which is connected (L1263) to a handler that writes the value
*back* into `LiveControl`. If we didn't block it, the controller would set the
slider, the slider would fire, the handler would overwrite the live command — a
**feedback loop** fighting the controller. Blocking signals makes this a
display-only update. Recognizing and cutting feedback loops like this is a core
GUI-programming instinct.

### 5f. The QTimer — pull instead of push for plots (L1042)

```python
self.plot_timer = QTimer(self)
self.plot_timer.timeout.connect(self.canvas.refresh)
self.plot_timer.start(50)   # fire every 50 ms
```
Telemetry arrives far faster than 20 times/second, and repainting is expensive.
Rather than redraw on *every* new sample (push), the GUI redraws on a fixed 50 ms
timer (pull), reading whatever's currently in the buffers. This **decouples data
arrival rate from render rate** — a standard technique to keep UIs smooth under a
firehose of data. The timer runs on the GUI thread (it's a Qt object), so
`refresh` touching the canvas is legal.

### 5g. The per-surface trim handlers — a closure factory (L1453)

Your per-surface trims (roll/pitch/yaw) need *three* nearly-identical slider
handlers that differ only in which axis they touch. Rather than write three
copy-pasted methods, the code uses a **factory** that manufactures a handler
bound to one axis:
```python
def _make_trim_handler(self, axis):        # L1453
    def _handler(value):
        value = int(value)
        for wdg in (self.trim_sliders[axis], self.trim_spins[axis]):
            if wdg.value() != value:
                wdg.blockSignals(True); wdg.setValue(value); wdg.blockSignals(False)
        self.worker.post("set_trim", (axis, value))
    return _handler
```
The inner `_handler` "remembers" `axis` even after `_make_trim_handler` has
returned — that captured-variable trick is a **closure**. Each call bakes a
different `axis` into a fresh function, so
`slider.valueChanged.connect(self._make_trim_handler("pitch"))` (L1212) wires a
pitch-only handler with no `if axis == ...` branching anywhere. This is the same
idea as the `default_factory` lambda in §4b — *a function that builds a
function* — and it's the clean way to turn "N copies that vary by one value" into
one parameterised generator.

Notice it reuses two patterns you've already met: the `blockSignals` fence from
§5e (so mirroring the slider into the spinbox doesn't re-fire) and the
`post("set_trim", (axis, value))` command from §3b (the write goes to the worker
thread, which owns the radio). The read-back on connect travels the other way,
via the `trim_value` signal (§5d) into `_on_trim_value` (L1472). Same four
mechanisms, new feature.

---

## 6. The plotting layer (matplotlib embedded in Qt, `PlotCanvas`, L271)

`matplotlib.use("QtAgg")` (L54) picks the backend that renders into a Qt widget.
`PlotCanvas` inherits from `FigureCanvasQTAgg`, so a matplotlib figure *is* a Qt
widget you can drop into a layout (L1131).

The key performance idea is in the split between `__init__` and `refresh`:
- In `__init__` (L272) each line is created **once**, empty: `self.ax.plot([], [])`.
  `.plot()` returns a list; `(self.line_gyroroll,) = ...` unpacks the single
  element (the trailing comma makes it tuple-unpacking of one item).
- In `refresh` (L312) we don't recreate anything — we just feed new numbers to
  the existing line objects with `set_data(xs, ys)`. Rebuilding the plot every
  frame would be far slower.

Then:
```python
for axis in (...):
    axis.relim()                                      # recompute data bounds
    axis.autoscale(enable=True, axis="both", tight=False)
self.draw_idle()
```
`relim` + `autoscale` re-fit the axes to the current data (this is the fix from
your earlier plot-cutoff bug). `draw_idle()` requests a repaint "when convenient"
rather than forcing an immediate one — it coalesces rapid updates. Note the
consumer side of the snapshot pattern: `refresh` calls `self.buffers.snapshot()`
(L313) once, then works purely on that copy.

---

## 7. The radio / cflib layer (talking to the aircraft)

### 7a. Connecting with a context manager (L473)

```python
with SyncCrazyflie(cfg.uri, cf=Crazyflie(rw_cache="./cache")) as scf:
    self.cf = scf.cf
    ...
    self._control_loop()
```
`SyncCrazyflie` is a **context manager** (like `open()`): entering the `with`
block opens the radio link; leaving it — *for any reason, including an
exception* — closes it. This guarantees the link is never left dangling. The
entire session lives inside this block; when `_control_loop` returns (because you
hit Disconnect), the `with` exits and tears down cleanly. A URI like
`radio://0/80/2M/E7E7E7E7E7` (L73) encodes radio index, channel, data rate, and
address.

### 7b. Parameters vs. logging — the two halves of the link

The Crazyflie exposes two systems, and understanding the split explains most of
this file's radio code:

- **Parameters** — settings *you push down* to the aircraft, by name:
  `self.cf.param.set_value("motorPowerSet.m1", value)`. Used for PID gains
  (L545), flight-mode config (L523), and your manual override (L845). Think
  "write a register."
- **Logging** — telemetry *the aircraft pushes up* to you. You declare a
  `LogConfig` listing which variables you want and how often (L571):
  ```python
  lg_motor = LogConfig(name="Motor", period_in_ms=self.config.period_motor_ms)
  lg_motor.add_variable("motor.m1", "uint16_t")   # L573, in a loop over names
  ```
  then register a **callback** that fires each time a packet arrives (L600):
  ```python
  self.log_configs["motor"].data_received_cb.add_callback(self._on_motor_log)
  ```
  Those callbacks (L613–638) run on **cflib's own threads** — which is exactly
  why they hand data off through the lock-guarded `PlotBuffers` instead of
  touching widgets. Everything connects back to §1's central question.

### 7c. The `try/except/finally` around the whole session (L465, L494, L497)

```python
try:
    ... connect, configure, run control loop ...
except Exception as exc:
    self._log(f"ERROR: {exc}\n")          # surface any failure to the GUI
    self.status.emit(f"Error: {exc}")
finally:
    self._shutdown()                       # ALWAYS runs
    self.connected.emit(False)
```
`finally` runs whether the block succeeded, failed, or returned — so cleanup
(disarm, close files, drop the link) is guaranteed. Catching `Exception` and
reporting it via a signal means a radio hiccup shows up as text in your Console
tab instead of silently killing the thread. This is **robust boundary handling**:
the risky outside world (radio) is wrapped so its failures can't crash the app.

### 7d. The control loop itself (L675)

```python
while not self._stop.is_set():
    if self.joystick is not None: # 0. refresh SDL joystick state ONCE per tick
        pygame.event.pump()       #    (see the note below on why this is here)
    self._drain_commands()        # 1. apply queued one-shot commands
    ... watchdog check ...        # 2. failsafe if telemetry stalls
    live = self._live_snapshot()  # 3. read a coherent copy of settings
    ... optional 1 Hz debug echo ...  # 3b. _debug_log_controller if enabled
    if live.manual_override:      # 4. decide what to command this tick
        self._drive_manual_override(live)
    elif self.config.use_controller:
        self._handle_controller_inputs(live)
    else:
        ... autonomous / hold-zero ...
    time.sleep(0.01)              # 5. ~100 Hz pace
```

**Why the `pygame.event.pump()` is at the very top (step 0).** SDL only updates a
joystick's axis/button values when its event queue is serviced; `get_axis()` just
returns the most recently pumped snapshot. Originally each *handler* pumped on its
own, which quietly coupled "are inputs read?" to "which branch ran / was the debug
echo on?" — flip the debug checkbox and the controller appeared to start/stop
working. Pumping exactly once per tick, unconditionally, decouples reads from every
code path: the stick is fresh whether you're in override, setpoint, or idle mode,
and whether or not the debug echo is enabled. The lesson: **a shared input source
should be refreshed in one well-defined place per cycle, not opportunistically
wherever a consumer happens to need it.**
`self._stop` is a `threading.Event` — a thread-safe boolean flag. The GUI's
`stop()` (L435) calls `self._stop.set()`; the loop notices and exits on its next
turn. This is the clean way to ask another thread to stop — you *never* forcibly
kill a thread, you ask it to finish its current work and return.

`time.sleep(0.01)` sets the loop's rhythm. Without it, the loop would spin as
fast as the CPU allows, pegging a core and flooding the radio — which connects
directly to the bug you just fixed.

### 7e. Your override fix, in this vocabulary (L783, L845)

The lesson from the bug you just fixed maps onto everything above:
- **Dirty-checking** (`_set_param_if_changed`, L845) caches the last value sent
  per parameter and skips the radio write if it's unchanged. The radio link is a
  scarce, slow resource; don't spend it re-sending identical data.
- **Non-blocking slew** (`_slew`, L839) moves the servo toward its target by a
  bounded step *each loop tick* instead of a blocking inner `for`-loop with
  `time.sleep`. Anything that blocks inside a loop that's supposed to run at a
  fixed rate wrecks that rate. The rule: **in a real-time loop, never block;
  advance state a little each tick and return.**

### 7f. Supporting more than one controller — the profile pattern (L399, L445)

The program can drive the aircraft from an Xbox pad *or* a GREAT PLANES
InterLink-X RC sim controller. These have completely different axis orders,
button counts, and even shapes of throttle (the Xbox steps throttle with the
D-pad; the RC unit has an absolute throttle stick). Rather than sprinkle
`if controller == "xbox"` branches through the control logic, the code uses a
**profile object** that *describes* the hardware, and the logic reads from the
description:

```python
@dataclass
class ControllerProfile:
    roll_axis: int; pitch_axis: int; yaw_axis: int; throttle_axis: int
    roll_sign: float = 1.0; ...            # flip if a surface moves backwards
    throttle_from_axis: bool = False       # True = absolute stick, False = D-pad steps
    has_hat: bool = True                   # False = no D-pad (skip get_hat)
    throttle_idle_raw: float = 1.0         # raw axis value at zero throttle
    throttle_full_raw: float = -1.0        # raw axis value at full throttle
    buttons: Dict[str, int] = field(default_factory=dict)  # command -> button index
```

`XBOX_PROFILE` and `RC_PROFILE` are two instances; the Setup-tab dropdown picks
one by key (`controller_type` in `SessionConfig`), and `start()` stashes it as
`self.profile`. Every read now goes through helpers that take indices *from the
profile*:
- `_axis(idx, sign)` (L983) reads one axis, returning `0.0` if the index is absent
  or out of range — so a wrong/missing index **silently disables** that control
  instead of crashing (this is what killed the first RC attempt: an unconditional
  `get_hat(0)` on a hatless device).
- `_edge_cmd("arm")` (L1018) looks up the button for a *logical command* in
  `profile.buttons`; a command with no button (e.g. breakpoint on a pad that's
  out of buttons) is simply skipped.
- `_throttle_norm(profile)` (L990) maps the throttle input to `0..1`. For an
  absolute stick it linearly rescales the calibrated raw range
  `[throttle_idle_raw .. throttle_full_raw]` so the stick's resting position is a
  *true zero* (motor off). This calibration is why the InterLink-X throttle — which
  idles at raw `+0.80`, not `0` — still commands 0% at rest.

This is the **strategy pattern**: the varying behavior is captured as data, and
one set of algorithms operates on it. Adding a third controller is a new
`ControllerProfile` and a dropdown entry — **no changes to the control logic**.
The `interlink_tester.py` helper script exists to *discover* those numbers on
real hardware (it prints each axis's index and rest value as you move the sticks),
which you then paste into the profile.

**Two safety behaviors worth calling out** (both in `_handle_command("arm")`, L832):
1. Arming no longer spins the motor. It used to force `throttle = max(throttle,
   0.7)` on arm; now arming only *enables* the throttle axis, and the motor stays
   at whatever the stick commands (idle = off).
2. An **arm interlock**: `_current_thrust_input()` is checked first, and arming is
   *refused* (with a logged `Arm REJECTED`) unless the throttle is at idle — so a
   spun-up throttle can never coincide with the moment the motor goes live.

The Setup-tab **"Log controller input"** checkbox flows into
`config.debug_controller_log` and only gates the 1 Hz `_debug_log_controller`
console echo — it has no effect on whether commands are sent (see §7d step 0 for
why that separation matters).

---

## 8. A day in the life of one click (tying it together)

Follow "you drag the throttle slider" through every layer:

1. Qt's event loop (GUI thread) detects the drag and emits `valueChanged`.
2. That's connected (L1156) to `_on_throttle_changed` (L1426), which updates a
   label and calls `self.worker.update_live(throttle=value/100.0)`.
3. `update_live` (L442) takes the worker's **lock** and writes the new throttle
   into the shared `LiveControl`. (GUI → worker, "current setting" channel.)
4. Meanwhile the worker thread's control loop (L675), on its own schedule, calls
   `_live_snapshot()` — takes the lock, copies `LiveControl` — and reads the new
   throttle.
5. It calls `_set_bl_motor_throttle`, which pushes a **parameter** down the radio.
6. The aircraft acts; its telemetry flows back up through a cflib **callback**,
   into the lock-guarded `PlotBuffers`.
7. Every 50 ms the **QTimer** fires `refresh`, snapshots the buffers, and repaints
   the graph — and a `telemetry` **signal** updates the status bar.

Every arrow between threads used one of the four safe mechanisms. No thread ever
reached across into another's data without a lock, a queue, or a signal. *That* is
the whole design in one sentence.

---

## 9. Practices worth stealing for your own code

- **One owner per piece of state.** Decide which thread owns each object; others
  reach it only through a lock, queue, or signal. Most threading bugs are
  violations of this single rule.
- **Hold locks briefly; snapshot and release.** Copy under the lock, then do slow
  work (drawing, radio I/O) on the copy.
- **Match the data structure to the data's shape.** Rolling window → `deque`;
  events → `queue`; settings → guarded object.
- **Wrap risky boundaries in `try/except/finally`.** Radio, files, hardware —
  assume they'll fail and make cleanup guaranteed.
- **Decouple rates.** Producers and consumers (telemetry vs. redraw) shouldn't be
  forced to run at the same speed; buffer between them and poll on a timer.
- **Never block a real-time loop.** Advance a little each tick.
- **Fail soft on optional deps.** `try/except ImportError` with a `None` sentinel.
- **Separate construction from logic.** One `_build_*` method per tab; short,
  named helpers over giant functions.

---

## 10. Mini-glossary

- **Thread** — an independent stream of execution within the process.
- **GIL** — CPython's lock allowing one thread to run bytecode at a time; released
  during I/O, which is why I/O-bound threading still helps.
- **Lock (mutex)** — makes a group of operations atomic across threads.
- **Atomic** — happens all-at-once from other threads' perspective; can't be seen
  half-done.
- **Race condition** — a bug where the result depends on unlucky thread timing;
  what locks/queues/signals prevent.
- **Context manager / `with`** — object that guarantees setup on entry and
  cleanup on exit (files, locks, the radio link).
- **Producer/consumer** — one thread creates data, another uses it, with a safe
  buffer between.
- **Deque / ring buffer** — fixed-size double-ended queue that drops oldest items.
- **Signal/slot** — Qt's observer system; also the safe worker→GUI thread bridge.
- **Decorator** (`@dataclass`) — a function that wraps/augments a class or
  function.
- **Callback** — a function you hand to a library to be called later when an event
  occurs (cflib log packets, button clicks).
- **Event loop** — the infinite GUI loop that dispatches events and repaints.
- **Sentinel** — a stand-in value (`None`) meaning "absent/not ready."
- **Closure / factory function** — an inner function that "remembers" a variable
  from the outer function that built it; used to stamp out N handlers that differ
  by one value (the per-surface trim handlers).
- **Persistence** — saving state to disk so it survives the program exiting;
  here, load-on-open + save-on-close for the Notes tab.

---

*Suggested way to study: open the file, put your cursor on `_control_loop`
(L675), and trace each branch outward — every path eventually touches one of the
four thread-crossing mechanisms. Once those four feel obvious, you understand this
program.*
```
