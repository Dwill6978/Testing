# Understanding `gui_GliderControlTest.py` — a guided tour

*Written for someone with a CS background who hasn't written serious code in a
while. I assume you remember what a variable, a function, a class, and a loop
are, but that terms like "context manager," "the GIL," or "signal/slot" are
fuzzy. We'll rebuild that vocabulary as we go, always tying it back to a real
line in your file.*

Read this next to the source. I cite line numbers like `(L1230)` so you can jump.
(Line numbers track the current file; if you edit the code they'll drift.)

---

## 0. The one-paragraph mental model

This program does four things: it **talks to a flying robot over a radio**, it
**draws live graphs**, it **listens to your mouse/keyboard/game controller**, and
— the newest half — it **reads flight logs back off disk and analyses them**. The
first three have wildly different timing needs, so the program splits them across
**threads** (independent streams of execution) and connects those threads with a
few carefully chosen **thread-safe hand-off mechanisms**. The fourth crosses a
different boundary entirely: not between threads, but between *runs of the
program*, which is why it's all files. Almost every design decision in this file
answers one of two questions: *"which thread is allowed to touch this piece of
data, and how does it cross safely?"* and *"what has to survive the process
exiting?"* Hold onto those two — they're the spine of the whole program.

---

## 1. Why threads at all? (the core theory)

A **thread** is a sequence of instructions the CPU runs. A normal Python script
has exactly one thread: line runs after line, top to bottom. That's fine until
you need to do two things that *both* want to "wait" or "loop forever":

- The **GUI** must run an infinite loop that watches for clicks and repaints the
  screen. If that loop ever stops, the window freezes ("Not Responding").
- The **control loop** (L1230, `_control_loop`) must run its own tight `while`
  loop ~100 times per second, sending commands to the aircraft and reading
  telemetry.

You cannot run two infinite loops in one thread — the first one never returns.
So the program uses **two threads**:

1. The **GUI thread** (also called the "main thread") — owns every widget and the
   matplotlib canvas.
2. The **worker thread** — owns the radio connection and the control loop.

Started at L767, inside `start()`:
```python
self._thread = threading.Thread(target=self._run, daemon=True)
self._thread.start()
```
`target=self._run` says "when this thread starts, call `self._run`." `.start()`
launches it running *alongside* the caller — control returns immediately to the
GUI while `_run` executes in parallel.

`daemon=True` means "if the whole program exits, don't wait for this thread —
just kill it." A non-daemon thread would keep Python alive after you close the
window. (There's still a clean shutdown path at L3293, `closeEvent`, which
politely asks the thread to stop and `join`s it — waits for it to finish — with a
timeout so a hung radio can't hang the app forever.)

Note the guard on the first two lines of `start()` (L761): if a thread is already
alive, it returns immediately. Without that, a double-click on Connect would
spawn two control loops both driving the same aircraft. **Making a start
operation idempotent is cheap insurance.**

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

There is also a **third** set of threads you don't create: cflib spawns its own
for radio traffic, and your log callbacks (L1167–1195) run on *those*. So the
program has GUI thread + worker thread + cflib's threads, which is exactly why
the buffers those callbacks write into are lock-guarded.

---

## 2. The file's skeleton (top-to-bottom map)

Skim these so you know the neighborhoods before we zoom in. The file has roughly
doubled since the first version of this guide; the growth is almost entirely in
two places — the worker's safety/tuning logic, and the offline Flight Data tab.

| Lines | What lives there | Role |
|-------|------------------|------|
| 1–24 | Module docstring | Human-readable overview |
| 26–78 | Imports | Tools; note the *try/except* fallbacks and `import flight_plots` |
| 80–217 | Constants + path helpers | Named magic numbers, the three persistence files, `resolve_log_prefix`, `clipped_day_dir` |
| 220–275 | PID dataclasses + JSON persistence + `clamp` | `PidGains`, `load_default_gains`, `save_default_gains` |
| 278–358 | `CsvLogBundle` | Writes telemetry, events and the console mirror to disk |
| 361–427 | `PlotBuffers` | **Thread-safe** telemetry hand-off |
| 430–535 | `PlotCanvas` | The embedded live matplotlib graphs |
| 537–583 | `SessionConfig`, `LiveControl` | Two kinds of shared state |
| 585–692 | `ControllerProfile`, `XBOX_PROFILE`, `RC_PROFILE` | Per-controller data profiles (§7f) |
| 695–1978 | `GliderWorker` | The worker thread: radio, control loop, failsafes, knob tuning |
| 1980–2003 | `BUTTON_MAPPING` | Static reference data |
| 2005–3298 | `MainWindow` | Every widget + GUI-side logic, including the Flight Data tab |
| 3301–3309 | `main()` + `__main__` guard | Program entry point |

Notice the ordering: **things are defined before they're used**. Python reads
top-to-bottom, so a class must exist before another class references it. The one
that runs *everything*, `main()`, is at the very bottom — because it needs all
the others to already be defined.

---

## 3. Data handling: how information moves without corrupting itself

This is the most important section. Five different data-transfer problems, five
different tools. Learning *why each tool fits its problem* is the real lesson.

### 3a. Continuous streaming data → `deque` + `Lock` (`PlotBuffers`, L364)

Telemetry pours in from the aircraft (gyro rates, motor commands, battery). The
graphs only ever need the *recent* history — a rolling window. The perfect data
structure is a **`deque`** ("deck," double-ended queue) created with a `maxlen`:

```python
d = lambda stream: deque(maxlen=self._maxlen(stream))   # L384
```
A `deque(maxlen=N)` automatically drops the oldest item when you append the
(N+1)th. So the buffer can never grow unbounded — memory stays flat no matter how
long you fly. That's a classic **ring buffer**, and `deque` gives it to you for
free. (`.append()` and the auto-eviction are also individually thread-safe, which
is a nice bonus, though we still lock — see below.)

`self._maxlen` (L379) computes the window size from *time*: if you want 20 s of
history and samples arrive every 50 ms, you need `20 / 0.050 = 400` slots. This
is why every graph shows the *same time span* even though different streams log
at different rates — each deque is sized for its own rate. `configure()` (L391)
rebuilds them all when you change periods in the Setup tab, and it takes the lock
to do it, because swapping the buffers out from under a live producer would be a
textbook race.

**The lock.** Look at every method that touches the deques:
```python
def add_connection(self, ts, rssi):
    with self._lock:                     # L412
        self.t_conn.append(ts); self.rssi.append(rssi)
```
`self._lock = threading.Lock()` (L374). `with self._lock:` is a **context
manager** (the `with` statement): it acquires the lock on entry and *guarantees*
release on exit, even if an exception is thrown inside. This is the same `with`
pattern you've seen for files (`with open(...)`), and it's the idiomatic way to
handle any "acquire → do work → release" resource in Python.

Why lock here? The **producer** is a cflib callback running on cflib's own thread
(L1167 `_on_controller_log` etc.), and the **consumer** is the GUI redraw timer
(L480 `refresh`). One writes while the other reads. Without the lock, the reader
could catch the writer mid-update and see half-written state, or the two
appends could interleave. This producer/consumer split across threads is a
textbook concurrency pattern, and the lock is what makes it safe.

**The snapshot trick (L420).** `refresh` doesn't hold the lock while it draws
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

`getattr(self, name)` is **reflection**: fetching an attribute *by its string
name* at runtime. It lets one line handle 19 buffers instead of 19 lines. The
`{k: v for ... in ...}` is a **dict comprehension** — same idea as a list
comprehension but building a dictionary.

### 3b. One-shot commands GUI → worker → `queue.Queue` (L773, L1304)

When you click "Arm Motor," the GUI thread must tell the worker thread to do
something *once*. You can't just call the worker's method directly — that would
run the arming code *on the GUI thread*, which doesn't own the radio. Instead the
GUI drops a message in a **thread-safe queue**:

```python
def post(self, action, payload=None):     # L773
    self._cmd_queue.put((action, payload))
```
`queue.Queue` is built for exactly this: multiple threads can `put`/`get` without
you writing any locks — the locking is inside. The worker drains it at the top of
every loop iteration:
```python
def _drain_commands(self):                # L1304
    while True:
        try:
            action, payload = self._cmd_queue.get_nowait()
        except queue.Empty:
            return
        self._handle_command(action, payload)
```
`get_nowait()` returns instantly; if the queue is empty it raises `queue.Empty`,
which we catch to break out. The `(action, payload)` **tuple** is a tiny message
format — a string naming the action plus optional data. `_handle_command` (L1312)
is one big `if/elif` chain that turns those strings back into radio calls. This is
the **command pattern**: actions are captured as data ("arm", payload) and
executed later, on the correct thread.

**Why a queue for commands but a lock for telemetry?** Commands are discrete
events that must each happen exactly once, in order — a queue preserves order and
delivers each item once. Telemetry is a continuously overwritten "latest value" —
a lock-guarded buffer fits that. Matching the tool to the data's *shape* is the
skill here.

**One trap this design has.** The queue is only drained *inside* the connected
control loop. Anything posted while disconnected sits there and gets applied on
the *next* connect. That's why `_save_pid_defaults` (L3180) deliberately does
**not** `post` an event — a host-side action shouldn't leave a message that lands
in a future session's log file. When you add a `post` call, ask "what happens if
nobody is listening yet?"

### 3c. Continuous control values GUI → worker → lock-guarded object (`LiveControl`, L566)

Some values aren't one-shot events *or* streams — they're "the current setting,"
like throttle position or the override slider values. The GUI updates them
whenever you move a widget; the worker reads the latest value every loop. That's
the `LiveControl` dataclass, guarded by the worker's lock:

```python
def update_live(self, **kwargs):          # L777
    with self._lock:
        for k, v in kwargs.items():
            setattr(self.live, k, v)
```
`**kwargs` ("keyword arguments") lets callers write
`update_live(throttle=0.5, motor_armed=True)` and receive them as a dict. The
loop `setattr`s each onto the shared object under the lock. The worker reads a
consistent copy with `_live_snapshot` (L783):
```python
def _live_snapshot(self):
    with self._lock:
        return LiveControl(**vars(self.live))
```
`vars(obj)` returns the object's attributes as a dict; `LiveControl(**that)`
rebuilds a fresh independent copy. Same "copy under lock" trick as the plot
buffers — the control loop reads one coherent snapshot per tick, so a value can't
change halfway through the loop's logic.

### 3d. State that must outlive the program → a plain text file (Notes tab, L3101)

Everything above crosses a *thread* boundary but lives and dies with the process:
close the app and it's gone. The Notes tab crosses a different boundary —
**time / separate launches**. The tool for that is the oldest one in the book:
read a file on startup, write it back on exit.

```python
NOTES_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "glider_notes.txt")  # L163
```
`__file__` is the path to *this script*; `os.path.dirname(os.path.abspath(...))`
turns it into "the folder this script lives in," so the notes file sits next to
the code no matter what directory you launch from. Building paths with
`os.path.join` instead of gluing strings with `"/"` is the portable habit — it
uses the right separator per OS and avoids double-slash bugs.

The load/save pair is deliberately tiny:
```python
def _load_notes(self):                      # L3101
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

`_save_notes` (L3112) is the mirror image. It's called from **two** places: the
"Save notes now" button (a manual flush, in case the app crashes) and
`closeEvent` (L3293), so a clean exit always persists.

### 3e. Structured settings that must outlive the program → JSON (L167, L171)

Notes are freeform text, so a `.txt` is the whole job. But two other things now
persist across launches, and they have *structure* — named fields with types:

- **`glider_pid_defaults.json`** (L167) — your tuned rate gains, twelve numbers.
- **`glider_setup_defaults.json`** (L171) — every Setup-tab control.

For structured data the right tool is **JSON**: a text format that round-trips
dicts, lists, numbers, strings and booleans. `json.dump(obj, fh, indent=2)` writes
it; `json.load(fh)` reads it back. `indent=2` costs nothing and makes the file
human-readable and diff-able — which matters, because both files are committed to
git, so `git log -p glider_pid_defaults.json` replays your tuning history.

**The PID pair (L242, L267)** shows the shape:
```python
def load_default_gains() -> PidGains:
    gains = PidGains()                       # start from the built-in defaults
    try:
        with open(PID_DEFAULTS_FILE, "r", encoding="utf-8") as fh:
            saved = json.load(fh)
    except FileNotFoundError:
        return gains
    except (OSError, ValueError):
        return gains
    for axis in PID_AXES:
        ...  # overwrite field by field, skipping anything malformed
    return gains
```
The critical design choice is that **loading never fails**. It starts from a valid
object and overlays whatever the file can supply, field by field. A missing file,
a corrupt file, a hand-edited typo, or a file written by an older version all
degrade to "use the built-in default for that one field" instead of throwing.

Why so defensive? Because this file is on the critical path of *starting the
program*. A settings file that can crash startup is a settings file that can
strand you at the flying field with a GUI that won't open. **The blast radius of
a parse error determines how much you invest in tolerating it.**

`save_default_gains` uses `asdict()` from `dataclasses`, which recursively turns
a dataclass into plain dicts — exactly what `json.dump` wants:
```python
payload = {axis: asdict(getattr(gains, axis)) for axis in PID_AXES}
```
Note it deliberately does *not* catch `OSError`; the caller does (L3186), so the
helper stays a pure "do the thing" function and the GUI decides how to report
failure. **Push error *presentation* to the layer that has a console to print to.**

The Setup-tab pair (L2220, L2232) works the same way but is driven by a binding
table — that's §5h, because the interesting part is the GUI structure, not the
file format.

> **Recap of the data-handling toolbox:**
> - Rolling stream of samples → `deque(maxlen)` + `Lock`, hand off by snapshot.
> - Discrete "do this once" events → `queue.Queue`.
> - "Current setting" values → a lock-guarded shared object.
> - Freeform text across runs → a plain `.txt`, load-on-open/save-on-close.
> - Structured settings across runs → JSON, loaded defensively field by field.
>
> Five problems, five tools. The first three cross *threads within one run*; the
> last two cross *runs across time*. Same mindset — "who owns this data and how
> does it get somewhere else safely" — pointed at a different boundary.

---

## 4. The `dataclass`es and other Python syntax refreshers

### 4a. `@dataclass` (L223, L231, L541, L566, L586)

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

Two dataclass superpowers this file leans on: `vars()`/`**` for the snapshot copy
(§3c), and `asdict()` for JSON serialisation (§3e). Bags-of-fields are boring by
design, and that's exactly what makes them easy to copy, compare and persist.

### 4b. `field(default_factory=...)` — the mutable-default trap (L232)

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

This one genuinely bites here: `load_default_gains` mutates the `PidGains()` it
just built. Without `default_factory` that mutation would leak into every future
`PidGains()` in the process.

### 4c. `lambda` (L232, L384, L490)

`lambda x: expr` is an anonymous one-line function. `lambda: PidAxisGains(...)`
takes no args and returns a new object. It's used where a throwaway function is
needed inline — as a factory here, as a deque builder at L384, as a small maths
helper (`defl = lambda v: ...` at L490), and all over the GUI for button handlers
(`lambda: self.worker.post("persist_trim")`, L2367).

### 4d. Generator expressions (L385)

```python
self.t_ctrl, self.gyroroll, self.gyropitch, self.gyroyaw = (d("controller") for _ in range(4))
```
`(expr for x in iterable)` is a **generator** — like a list comprehension but it
produces items lazily, one at a time, instead of building a whole list. Here it
yields four fresh deques, which are unpacked into four variables. The `_` is the
conventional name for "a loop variable I don't care about."

### 4e. `Optional[...]`, `Tuple[...]`, `Dict[...]` (L41)

`Optional[LogConfig]` means "a `LogConfig` **or** `None`." It documents that a
variable is legitimately allowed to be empty (e.g. `self.cf = None` before we
connect). `Tuple[int, int]` means "a 2-tuple of ints"; `Dict[str, int]` a dict
from strings to ints. These come from the `typing` module and, again, are
documentation, not enforcement. `_catch_value` returning `Optional[float]` (L1645)
is a good example of a type hint carrying real meaning: `None` is the "not caught
yet, don't act on this" signal, and the hint tells you to expect it.

### 4f. The import fallbacks (L46, L74) — defensive design

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
file can just say `Signal`. Same idea at L74 for pygame: if it's not installed,
`pygame = None`, and later code checks `if pygame is None`. This is **graceful
degradation** — the program still runs (minus the controller) instead of crashing
on an import.

### 4g. `@staticmethod` (L1477, L1813, L2188, L2201)

```python
@staticmethod
def _widget_value(wdg): ...
```
A **static method** lives inside the class for organisational reasons but doesn't
receive `self` — it doesn't touch instance state. `_slew` (L1477) is pure maths;
`_widget_value` only looks at the widget you hand it. Marking them static is a
small honesty signal: *this function can't secretly depend on the object*, which
makes it trivially testable and safe to call from anywhere.

### 4h. `isinstance` dispatch (L2188)

```python
if isinstance(wdg, QtWidgets.QCheckBox):
    return wdg.isChecked()
if isinstance(wdg, QtWidgets.QComboBox):
    return wdg.currentData()
```
Qt has no single "give me your value" method — checkboxes, combos, line edits and
spin boxes each spell it differently. Rather than store a getter alongside every
widget, one function branches on the widget's **type**. This keeps the binding
table (§5h) a dumb `{name: widget}` dict instead of `{name: (widget, get, set)}`
triples. Order matters: check `QSpinBox` before the generic fallback, because the
int/float distinction is real (§5h).

---

## 5. The GUI layer (Qt): event loops, signals, and slots

### 5a. The event loop and `main()` (L3301)

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

### 5b. `if __name__ == "__main__":` (L3308)

This guard means "only run `main()` if this file was executed directly, not if it
was `import`ed by another file." `__name__` is `"__main__"` when you run
`python gui_GliderControlTest.py`, but becomes the module's name when imported.
It's the standard way to make a file usable both as a program and as a library.

### 5c. Widgets, layouts, and tabs (`MainWindow`, L2008)

`MainWindow` **inherits** from `QtWidgets.QMainWindow` (`class MainWindow(QMainWindow)`).
`super().__init__()` (L2010) calls the parent's constructor — you must do this so
Qt sets up its internal machinery before you add your own widgets.

The UI is built from **widgets** (buttons, sliders, labels) arranged by
**layouts**. A layout is an invisible manager that positions and resizes its
children automatically. You'll see:
- `QVBoxLayout` — stacks children vertically.
- `QHBoxLayout` — in a row.
- `QFormLayout` — label/field pairs (L2065).
- `QGridLayout` — a table of rows/columns (L2315 PID grid, L2344 trims).
- `QSplitter` (L2551) — a *draggable* divider; unlike a layout it lets the user
  re-allocate space at runtime, which is why the Flight Data tab uses one.

Each `_build_*_tab` method (L2063, L2265, L2272, L2410, L2514, L2531, L3063,
L3075) constructs one tab's widget tree and returns the top widget, which is added
to the `QTabWidget` (L2032–2041). There are now **eight** tabs — Setup, Live
Plots, Control, Manual Override, Mapping, Flight Data, Console, and Notes.
Splitting UI construction into one method per tab keeps `__init__` readable — a
good **separation-of-concerns** habit, and the reason a 1,300-line `MainWindow`
is still navigable.

`_scrollable` (L2057) wraps a tab in a `QScrollArea` so tall tabs stay usable on a
small screen. Note which tabs *aren't* wrapped: Live Plots, Flight Data and
Console want to consume all available space rather than scroll.

**Startup ordering matters (L2032–2054).** The tabs are built, *then* the plot
timer starts, *then* `_load_setup_defaults()` runs, *then* `_set_connected_ui(False)`.
That order is deliberate: loading settings can print a warning to the Console
pane, so the Console tab must already exist. Get it backwards and a corrupt
settings file crashes the program on launch instead of reporting itself. **When a
startup step can fail, make sure the thing that reports failures is built first.**

### 5d. Signals and slots — Qt's event system (and its threading superpower)

Qt objects emit **signals** ("something happened") that connect to **slots**
(functions to run in response). You wire them with `.connect`:
```python
self.connect_btn.clicked.connect(self._toggle_connection)      # L2154
self.throttle_slider.valueChanged.connect(self._on_throttle_changed)
```
"When the button is clicked, call `_toggle_connection`." This is the **observer
pattern**: the button doesn't know who's listening; anyone can subscribe.

The truly important part: at the top of `GliderWorker` (L699–707) *custom* signals
are declared:
```python
console = Signal(str)
telemetry = Signal(float, float)
override_state = Signal(int, int, int, int, int)
trim_value = Signal(str, int)          # (axis, value) read back from the deck
surface_map = Signal(str, int, int)    # (channel, surface_code, invert)
pid_value = Signal(str, str, float)    # (axis, term, value) live in-flight tune
```
The worker thread **emits** these (e.g. `self.telemetry.emit(vbat, rssi)`), and
the GUI connects them to its own methods (L2022–2027). **Why this matters:** it is
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
worker thread --Signal.emit--> GUI thread          (console/status/knob echo)
```

Notice the last three signals all exist for the same reason: something can now be
changed **on the aircraft or the transmitter**, not just in the GUI, and the GUI
has to catch up. Which leads directly to:

### 5e. The `blockSignals` fence — cutting feedback loops (L2497, L3214, L3235, L3248)

Every "reflect a value read back from elsewhere" handler shares one subtle line:
```python
def _on_override_state(self, m1, m2, m3, m4, servo):     # L2497
    for name, value in (...):
        slider = self.override_sliders[name]
        slider.blockSignals(True)      # <-- the subtle, important line
        slider.setValue(int(value))
        slider.blockSignals(False)
```
Why `blockSignals(True)`? Setting a slider's value normally **emits**
`valueChanged`, which is connected to a handler that writes the value *back* into
`LiveControl`. If we didn't block it, the controller would set the slider, the
slider would fire, the handler would overwrite the live command — a **feedback
loop** fighting the controller.

This pattern now appears four times (override sliders, trim readback, surface-map
readback, live PID tune echo) because four different things can change a widget
from outside. **The rule: any time you set a widget programmatically to *display*
a value rather than to *command* one, block its signals.** Recognizing and cutting
feedback loops like this is a core GUI-programming instinct.

### 5f. The QTimer — pull instead of push for plots (L2046)

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

### 5g. Closure factories — N handlers that differ by one value (L3195, L2468, L3225)

The per-surface trims (roll/pitch/yaw) need *three* nearly-identical slider
handlers that differ only in which axis they touch. Rather than write three
copy-pasted methods, the code uses a **factory** that manufactures a handler bound
to one axis:
```python
def _make_trim_handler(self, axis):        # L3195
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
`slider.valueChanged.connect(self._make_trim_handler("pitch"))` wires a pitch-only
handler with no `if axis == ...` branching anywhere. Same idea as the
`default_factory` lambda in §4b — *a function that builds a function* — and it's
the clean way to turn "N copies that vary by one value" into one parameterised
generator. The file uses it three times: trims, override sliders/spins (L2468,
L2476), and surface-map channels (L3225).

Notice it reuses two patterns you've already met: the `blockSignals` fence from
§5e and the `post("set_trim", ...)` command from §3b (the write goes to the worker
thread, which owns the radio). The read-back travels the other way, via the
`trim_value` signal (§5d) into `_on_trim_value` (L3214). Same mechanisms, new
feature.

### 5h. The binding table — one map, four behaviors (L2158)

This is the newest structural idea in the file, and the most reusable.

The Setup tab's widgets correspond 1:1 with `SessionConfig`'s fields. But that
correspondence used to be written out **three separate times**: creating the
widgets, reading them into a config, and listing them to grey out while connected.
Adding save + restore would have made it **five parallel lists** — and the failure
mode of parallel lists is guaranteed and boring: you add a setting, update four of
five, and lose an evening wondering why one control won't persist.

The fix isn't to add lists. It's to collapse them:
```python
def _setup_bindings(self) -> Dict[str, QtWidgets.QWidget]:   # L2158
    return {
        "uri": self.uri_edit,
        "use_controller": self.controller_chk,
        "fwactlpf_cutoff_hz": self.lpf_cutoff_spin,
        "log_motor": self.log_motor_chk,
        "period_accelerometer_ms": self.period_accel_spin,
        ...   # 19 entries, keyed by SessionConfig field name
    }
```
Four behaviors now derive from that one map:

1. **Collect the config** (L3129) — `{field: self._widget_value(w) ...}` then
   `SessionConfig(gains=..., **values)`. The `**` splat turns the dict into keyword
   arguments, which is why the keys *must* be exact field names.
2. **Save defaults** (L2220) — the same dict, straight to `json.dump`.
3. **Restore defaults** (L2232) — walk the map, set each widget from the file.
4. **Lock while connected** (L3278) — `for wdg in self._setup_bindings().values()`,
   which replaced a nine-line hand-written widget list.

Adding a Setup setting is now a two-step job (build the widget, add one map line)
instead of a five-step job with four chances to forget.

The generic accessors (§4h) are what make the map able to stay a dumb
`{name: widget}` dict. Two details worth stealing:

- **Combos save `currentData()`, not the label.** The stored value is the stable
  key `"xbox"`, so renaming the dropdown text to "Xbox / gamepad (USB)" doesn't
  invalidate every saved file. **Persist identifiers, never display strings.**
- **JSON has no int/float distinction.** A period saved as `50` reads back as
  `int`, but `50.0` would read back as `float`, and `QSpinBox.setValue` rejects a
  float. Hence the explicit `int()` / `float()` split in `_set_widget_value`
  (L2201). Serialisation formats are lossier than your type system; coerce at the
  boundary.

Restoring is as forgiving as the PID loader (§3e): unknown fields are ignored,
bad values are skipped field-by-field with a console note, an unknown combo key
leaves the selection alone rather than snapping it to index 0, and a non-dict
top-level JSON value is simply dropped. Nine malformed-input cases were tested;
none can prevent the GUI from launching.

**A note on what is deliberately *not* in the map:** the "Save settings as launch
defaults" button itself. Because the lock loop iterates the map, anything absent
from it stays enabled while connected — which is exactly what you want for that
button, so a session's settings can be blessed after connecting. A structure that
makes the right thing happen by default is better than one that needs a special
case.

---

## 6. The live plotting layer (matplotlib embedded in Qt, `PlotCanvas`, L433)

`matplotlib.use("QtAgg")` (L61) picks the backend that renders into a Qt widget.
`PlotCanvas` inherits from `FigureCanvasQTAgg`, so a matplotlib figure *is* a Qt
widget you can drop into a layout.

The key performance idea is in the split between `__init__` and `refresh`:
- In `__init__` (L434) each line is created **once**, empty: `self.ax.plot([], [])`.
  `.plot()` returns a list; `(self.line_gyroroll,) = ...` unpacks the single
  element (the trailing comma makes it tuple-unpacking of one item).
- In `refresh` (L480) we don't recreate anything — we just feed new numbers to
  the existing line objects with `set_data(xs, ys)`. Rebuilding the plot every
  frame would be far slower.

Then `relim()` + `autoscale()` re-fit the axes to the current data, and
`draw_idle()` requests a repaint "when convenient" rather than forcing an
immediate one — it coalesces rapid updates. Note the consumer side of the snapshot
pattern: `refresh` calls `self.buffers.snapshot()` once (L481), then works purely
on that copy.

**Deflection, not raw counts (L490).** Motor channels are plotted through
`defl = lambda v: clamp((v - SERVO_TRIM_CENTER) / SERVO_TRIM_CENTER, -1.0, 1.0)`,
converting a raw 0–65535 command into a −1..+1 deflection about centre. Plotting
the *physically meaningful* quantity rather than the wire value is what makes a
graph readable at a glance.

**Labels follow the mixer (L515, L521).** `set_surface_map` receives the current
channel→surface assignment and `_apply_surface_map_labels` relabels and recolours
the traces (Aileron/Elevator/Rudder), hiding any channel not mapped to a surface.
So if you remap M3 from rudder to elevator in the Mapping tab, the legend follows.
**A plot legend that can silently disagree with reality is worse than no legend;**
driving it from the same data the aircraft uses removes that whole class of error.

---

## 7. The radio / cflib layer (talking to the aircraft)

### 7a. Connecting with a context manager (L809)

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
`radio://0/80/2M/E7E7E7E701` (L83) encodes radio index, channel, data rate, and
address.

### 7b. Parameters vs. logging vs. packets — three ways to talk

The Crazyflie exposes three channels, and knowing which is which explains most of
this file's radio code:

- **Parameters** — named settings *you push down*:
  `self.cf.param.set_value("pid_rate.pitch_kp", value)`. Used for PID gains
  (L1097), flight-mode config (L1060), trims, and the surface mixer. Think "write
  a register." Reliable and acknowledged, but relatively slow — which is why
  writes are dirty-checked (§7e).
- **Logging** — telemetry *the aircraft pushes up*. You declare a `LogConfig`
  listing which variables you want and how often (L1119), then register a
  **callback** that fires each time a packet arrives (L1150). Those callbacks
  (L1167–1195) run on **cflib's own threads** — which is exactly why they hand data
  off through the lock-guarded `PlotBuffers` instead of touching widgets.
- **Raw CRTP packets** — a hand-built binary message for the high-rate path
  (L1483, §7g).

### 7c. The `try/except/finally` around the whole session (L797)

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

`_shutdown` (L1922) is worth reading as a checklist of "what must be true when we
leave": motors disarmed, override off, setpoints zeroed, logs flushed and closed.

### 7d. The control loop itself (L1230)

```python
while not self._stop.is_set():
    if self.joystick is not None:      # 0. refresh SDL state ONCE per tick
        pygame.event.get()
    self._drain_commands()             # 1. apply queued one-shot commands
    ... connection watchdog ...        # 2. failsafe if telemetry stalls
    if use_controller: self._poll_controller_buttons(...)   # 3. discrete inputs
    live = self._live_snapshot()       # 4. coherent copy of settings
    ... optional 1 Hz debug echo ...
    if live.manual_override:           # 5. one continuous path, chosen by mode
        self._drive_manual_override(live); self._feed_supervisor_keepalive()
    elif self.config.use_controller:
        self._handle_controller_flight(live)
    else:
        ... autonomous setpoints or hold-zero ...
    time.sleep(0.01)                   # 6. ~100 Hz pace
```

**Why `pygame.event.get()` is at the very top (step 0).** SDL only updates a
joystick's axis/button values when its event queue is serviced; `get_axis()` just
returns the most recently pumped snapshot. Originally each *handler* pumped on its
own, which quietly coupled "are inputs read?" to "which branch ran / was the debug
echo on?" — flip the debug checkbox and the controller appeared to start/stop
working. Note it uses `get()` rather than a bare `pump()`: `get()` **drains** the
queue, and a moving stick floods it with `JOYAXISMOTION` events — if it's never
emptied, SDL stops updating and axis reads freeze at their last value (buttons,
being rare, still sneak through, which makes the bug maddeningly partial). The
lesson: **a shared input source should be refreshed — and drained — in exactly one
well-defined place per cycle.**

**Why discrete inputs are read before the mode branch (step 3).** Buttons and
latches are polled *unconditionally*, ahead of the `if live.manual_override`
split. If they were read inside a branch, the very switch that *leaves* that mode
would only be sampled while you were already in it — a state machine that can
enter a state it can't exit. **Read the inputs that change the mode outside the
code that depends on the mode.**

`self._stop` is a `threading.Event` — a thread-safe boolean flag. The GUI's
`stop()` (L770) calls `self._stop.set()`; the loop notices and exits on its next
turn. This is the clean way to ask another thread to stop — you *never* forcibly
kill a thread, you ask it to finish its current work and return.

`time.sleep(0.01)` sets the loop's rhythm. Without it, the loop would spin as fast
as the CPU allows, pegging a core and flooding the radio.

### 7e. Spending the radio budget wisely (L1477, L1506)

- **Dirty-checking** (`_set_param_if_changed`, L1506) caches the last value sent
  per parameter and skips the radio write if it's unchanged. The radio link is a
  scarce, slow resource; don't spend it re-sending identical data.
- **Non-blocking slew** (`_slew`, L1477) moves the servo toward its target by a
  bounded step *each loop tick* instead of a blocking inner `for`-loop with
  `time.sleep`. Anything that blocks inside a loop that's supposed to run at a
  fixed rate wrecks that rate. The rule: **in a real-time loop, never block;
  advance state a little each tick and return.**
- **Rate limiting** (`OVERRIDE_WRITE_INTERVAL`, L113) caps override writes at
  60 Hz even though the loop runs at 100 Hz.

### 7f. Supporting more than one controller — the profile pattern (L586, L634)

The program can drive the aircraft from an Xbox pad *or* a GREAT PLANES
InterLink-X RC sim controller. These have completely different axis orders, button
counts, and even shapes of throttle (the Xbox steps throttle with the D-pad; the
RC unit has an absolute throttle stick). Rather than sprinkle
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
`self.profile`. Every read goes through helpers that take indices *from the
profile*:
- `_axis(idx, sign)` (L1553) reads one axis, returning `0.0` if the index is absent
  or out of range — so a wrong/missing index **silently disables** that control
  instead of crashing (this is what killed the first RC attempt: an unconditional
  `get_hat(0)` on a hatless device).
- `_axis_c(idx, sign)` (L1560) subtracts the *measured* rest centre sampled at
  connect (`_sample_axis_centers`, L1039), so a stick that doesn't rest at exactly
  zero still commands zero.
- `_edge_cmd("arm")` (L1597) looks up the button for a *logical command* in
  `profile.buttons`; a command with no button is simply skipped. `_latch_edge`
  (L1609) is its cousin for switches, reporting both "changed" and "new level."
- `_throttle_norm(profile)` (L1569) maps the throttle input to `0..1`, linearly
  rescaling the calibrated raw range so the stick's resting position is a *true
  zero*. This is why the InterLink-X throttle — which idles at raw `+0.80`, not
  `0` — still commands 0% at rest.

This is the **strategy pattern**: the varying behavior is captured as data, and one
set of algorithms operates on it. Adding a third controller is a new
`ControllerProfile` and a dropdown entry — **no changes to the control logic**.
`interlink_tester.py` exists to *discover* those numbers on real hardware.

**Getting the USB device open is its own saga (L839–1038).** `_soft_replug` runs a
helper script to re-enumerate the device; `_is_streaming` (L878) *verifies* the
joystick is actually producing changing values rather than trusting that opening
it worked; `_open_streaming_joystick` (L894) retries until it is. This exists
because Parallels drops USB passthrough intermittently. **When a resource can open
successfully but be dead, "did it open?" is the wrong health check — test that it
actually does its job.**

### 7g. Hand-rolled packets and `struct` (L1483)

The manual-override path needs to send five motor values ~60×/second. That's too
much for the parameter system, so it builds a raw CRTP packet:
```python
pk.data = struct.pack("<BHHHHH", MANUAL_MOTOR_SETPOINT_TYPE, m1, m2, m3, m4, servo)
```
`struct.pack` converts Python values into a **byte string** with an exact binary
layout. The format `"<BHHHHH"` reads: `<` little-endian, `B` one unsigned byte
(the packet type), then five `H` unsigned 16-bit ints. This must match
`manualMotorPacket_s` in the firmware *exactly* — one wrong letter and the
aircraft interprets garbage. Every value is `clamp`ed to `0..MAX_MOTOR_CMD` first,
because a value that overflows `H` would wrap silently to something small.

**This is a real contract with code in another repository, enforced by nothing but
a comment.** When you next touch the firmware struct, this line is the thing that
breaks. Note it, and keep the comment at L1486 accurate.

### 7h. Failsafes: three independent watchdogs (L1197, L1205, L1288)

Nothing in this section is about features; it's all about what happens when
something goes wrong while a propeller is spinning.

1. **Link callbacks (L1197).** `connection_lost`, `connection_failed` and
   `disconnected` all route to `_failsafe_disarm`.
2. **Telemetry watchdog (L1245).** Even if cflib thinks the link is fine, if no
   connection packet has arrived for `CONNECTION_WATCHDOG_TIMEOUT_S` (1 s), the
   loop disarms. This catches a link that's silently dead rather than formally
   closed.
3. **Supervisor keepalive (L1288).** This one runs the *other* way — it prevents a
   firmware failsafe from firing spuriously. The manual-override path sends no
   commander setpoints, so the firmware's 2 s setpoint watchdog would block the
   commander. The watchdog checks setpoint *age*, not value, so sending a zero
   setpoint is enough to pet it, and it doesn't fight the override because
   `motorPowerSet.enable` overrides the ratios.

`_failsafe_disarm` (L1205) is worth studying as a piece of defensive writing:
```python
if self.failsafe_active:
    return                      # idempotent: only fire once
self.failsafe_active = True
self.update_live(motor_armed=False, autonomous=False, throttle=0.0, manual_override=False)
if self.logs is not None:
    self.logs.write_breakpoint("FAILSAFE_DISARM")
try:  ... commander stop ...  except Exception: pass
try:  ... param writes, arming request False ...  except Exception: pass
```
Three things to notice. It's **idempotent** (a re-entry guard, since three
callbacks can fire at once). It updates the *local* state first, so even if every
radio call fails the loop won't re-command a throttle. And each radio attempt is
in its own `try/except: pass` — normally a code smell, but here the alternative is
that a failed first call prevents the second from ever running. **In an emergency
path, "try everything, skip what fails" beats "stop at the first error."**

### 7i. Knob tuning, catch/takeover, and safety interlocks (L1637, L1719)

You can tune trims and PID gains from the transmitter's rear knobs mid-session.
That creates a classic problem: a physical knob has a position, and the stored
value it's about to control has a *different* one. Take control naively and the
gain snaps violently to wherever the knob happens to sit.

The fix is **catch/takeover** (borrowed from hardware synthesizers). On entering a
mode, `_arm_catch` (L1637) records which side of the stored value the knob is on.
`_catch_value` (L1645) then refuses to return a value — returning `None` — until
the knob has either come within `KNOB_CATCH_FRACTION` (2%) of the stored value or
*crossed past* it. Only then does the knob "grab" the parameter:
```python
val = self._catch_value(key, idx, lo, hi, stored)
if val is None:
    continue                    # not caught yet: knob moves, value doesn't
```
That `Optional[float]` return (§4e) is doing real work — `None` means "no opinion
yet," which is different from "zero."

`_poll_tune_modes` (L1719) wraps this in safety rules that are worth reading as a
model of interlock design:
- PID mode is **refused while armed**, logged as `PID_TUNE_REJECTED_ARMED`.
- If the motor becomes armed *while* PID mode is live, the mode drops out
  (`PID_TUNE_EXIT_ARMED`) — the invariant is enforced continuously, not just at
  the entry point.
- Recovery requires an explicit **off→on re-flip** of the switch, so a switch left
  in the "on" position can't silently re-enter tuning the moment you disarm.
- Changing the selected axis re-arms the catch, so switching from pitch to roll
  can't jump roll's gain to pitch's knob position.

A matching interlock guards arming itself (L1312, `_handle_command("arm")`):
arming is *refused* unless the throttle is at idle, so a spun-up throttle can
never coincide with the motor going live. And arming no longer forces throttle to
70% — it only *enables* the throttle axis.

**The general lesson:** an interlock that's only checked on entry is a suggestion.
Check the invariant every tick, and require a deliberate gesture to re-enter.

---

## 8. The offline half: the Flight Data tab (L2531)

Everything above is about a live aircraft. This tab is about the logs afterwards —
browse a day's sessions, plot them, cut the flights out of a long recording, and
push the results to OneDrive for the MATLAB pipeline on the Mac.

### 8a. Reusing your analysis scripts as libraries (L65)

```python
import flight_plots
```
`flight_plots.py` and `clip_flights.py` are standalone command-line tools you can
run in a terminal. The GUI doesn't shell out to them or duplicate their logic — it
**imports** them and calls their functions directly (`flight_plots.cf` is
`clip_flights`, re-exported). So `_write_clip_files` (L2892) calls
`flight_plots.cf._clip_stream(...)`, the exact code the CLI uses.

This is the payoff of the `if __name__ == "__main__":` guard (§5b): a file written
with a `main()` behind that guard is *simultaneously* a program and a library. One
implementation of flight detection, used from two front-ends — so a fix to the
detection thresholds improves the CLI and the GUI at once, and they can never
disagree about where a flight starts.

### 8b. Files as the interface (L190, L204)

Two small helpers impose the whole storage convention:
- `resolve_log_prefix` (L190) routes a session's files into `logs/YYYYMMDD/`,
  creating the folder if needed.
- `clipped_day_dir` (L204) mirrors that layout under `clipped/`, parsing the day
  back out of a `flight_YYYYMMDD_...` session name and falling back to
  `Misc_Flights` for anything unstamped.

`os.makedirs(day_dir, exist_ok=True)` is the idiom worth remembering: create the
directory, and don't complain if it's already there. The `exist_ok` flag replaces
a check-then-create race.

Note `CSV_SCHEMA_VERSION` (L187). Log files outlive the code that wrote them, so
stamping the format lets a future reader detect an old layout instead of
misinterpreting it. The same instinct explains why new CSV columns are appended at
the *end* — old files stay readable, and column indices don't shift under existing
parsers.

### 8c. Interactive matplotlib (L2828–2891)

Unlike the live canvas (§6), these figures are for exploration: `_fd_on_scroll`
(L2863) wires mouse-wheel zoom, `_fd_autoscale_y` (L2834) rescales Y to whatever X
range you're viewing, and a `NavigationToolbar` gives pan/zoom/save. Different job,
different trade-off — the live plot optimises for redraw speed, this one optimises
for inspection.

### 8d. Guard rails around destructive-ish operations

`_fd_sync_onedrive` (L2915) *replaces* same-named day folders at the destination,
so it checks first that the Parallels share is actually mounted:
```python
share_root = "/media/psf/Home"
if not os.path.isdir(share_root):
    self.fd_console.appendPlainText("Sync to OneDrive: the Parallels Home share isn't mounted ...")
    return
```
Without that check, an unmounted share would look like an empty destination and
the code would happily create a real local folder at that path — silently writing
your flight data into a directory that never syncs anywhere. It also filters to
strictly-named day folders with `re.fullmatch(r"\d{8}|Misc_Flights", ...)` rather
than copying whatever happens to be there.

Similarly `_write_clip_files` (L2892) returns a count of *dense data rows*, and
the caller uses that to warn you when a manual clip captured no actual flight
data. **When an operation can plausibly do nothing useful, make it report how much
it did rather than just "done."**

---

## 9. A day in the life of one click (tying it together)

Follow "you drag the throttle slider" through every layer:

1. Qt's event loop (GUI thread) detects the drag and emits `valueChanged`.
2. That's connected to `_on_throttle_changed` (L3155), which updates a label and
   calls `self.worker.update_live(throttle=value/100.0)`.
3. `update_live` (L777) takes the worker's **lock** and writes the new throttle
   into the shared `LiveControl`. (GUI → worker, "current setting" channel.)
4. Meanwhile the worker thread's control loop (L1230), on its own schedule, calls
   `_live_snapshot()` — takes the lock, copies `LiveControl` — and reads the new
   throttle.
5. It calls `_set_bl_motor_throttle` (L1539), which pushes a **parameter** down the
   radio (dirty-checked, so an unchanged value costs nothing).
6. The aircraft acts; its telemetry flows back up through a cflib **callback** on
   *cflib's* thread, into the lock-guarded `PlotBuffers`.
7. Every 50 ms the **QTimer** fires `refresh`, snapshots the buffers, and repaints
   the graph — and a `telemetry` **signal** updates the status bar.
8. In parallel, `CsvLogBundle` has been writing every sample to disk. Days later
   the Flight Data tab imports `clip_flights`, finds the flight in that recording,
   and writes a trimmed copy to `clipped/YYYYMMDD/`.

Every arrow between threads used one of the four safe mechanisms; the last step
crossed the fifth boundary — time. No thread ever reached across into another's
data without a lock, a queue, or a signal. *That* is the whole design in one
sentence.

---

## 10. Practices worth stealing for your own code

- **One owner per piece of state.** Decide which thread owns each object; others
  reach it only through a lock, queue, or signal. Most threading bugs are
  violations of this single rule.
- **Hold locks briefly; snapshot and release.** Copy under the lock, then do slow
  work (drawing, radio I/O) on the copy.
- **Match the data structure to the data's shape.** Rolling window → `deque`;
  events → `queue`; settings → guarded object; cross-run structure → JSON.
- **Collapse duplication instead of extending it.** When a fact is spelled out in
  three places, the fix for needing it in a fourth is one binding table, not a
  fourth list (§5h).
- **Persist identifiers, not display strings.** Saved files should survive a UI
  rewording.
- **Make loading unfailable when it's on the startup path.** Start from valid
  defaults, overlay field by field, degrade per-field on garbage.
- **Wrap risky boundaries in `try/except/finally`.** Radio, files, hardware —
  assume they'll fail and make cleanup guaranteed.
- **In emergency paths, try everything and skip what fails.** The normal rule
  ("don't swallow exceptions") inverts when the goal is to stop a motor.
- **Make dangerous operations idempotent.** Re-entry guards on failsafe and start.
- **Check invariants continuously, not just on entry.** An interlock tested only
  at the door is a suggestion (§7i).
- **Read mode-changing inputs outside the mode-dependent code.** Otherwise you can
  enter a state you can't leave.
- **Refresh a shared input source exactly once per cycle,** in one defined place.
- **Health-check that a resource *works*, not that it opened.**
- **Decouple rates.** Producers and consumers shouldn't be forced to run at the
  same speed; buffer between them and poll on a timer.
- **Never block a real-time loop.** Advance a little each tick.
- **Block signals when setting a widget for display, not for command.**
- **Fail soft on optional deps.** `try/except ImportError` with a `None` sentinel.
- **Write scripts that are both programs and libraries** (`__main__` guard), so
  one implementation serves the CLI and the GUI.
- **Separate construction from logic.** One `_build_*` method per tab; short,
  named helpers over giant functions.
- **When restructuring working code, prove equivalence.** Run the old and new
  implementations side by side and diff their output.

---

## 11. Mini-glossary

- **Thread** — an independent stream of execution within the process.
- **GIL** — CPython's lock allowing one thread to run bytecode at a time; released
  during I/O, which is why I/O-bound threading still helps.
- **Lock (mutex)** — makes a group of operations atomic across threads.
- **Atomic** — happens all-at-once from other threads' perspective; can't be seen
  half-done.
- **Race condition** — a bug where the result depends on unlucky thread timing;
  what locks/queues/signals prevent.
- **Idempotent** — safe to do twice; the second call changes nothing.
- **Context manager / `with`** — object that guarantees setup on entry and cleanup
  on exit (files, locks, the radio link).
- **Producer/consumer** — one thread creates data, another uses it, with a safe
  buffer between.
- **Deque / ring buffer** — fixed-size double-ended queue that drops oldest items.
- **Signal/slot** — Qt's observer system; also the safe worker→GUI thread bridge.
- **Decorator** (`@dataclass`, `@staticmethod`) — a function that wraps/augments a
  class or function.
- **Callback** — a function you hand to a library to be called later when an event
  occurs (cflib log packets, button clicks).
- **Event loop** — the infinite GUI loop that dispatches events and repaints.
- **Sentinel** — a stand-in value (`None`) meaning "absent/not ready" — e.g.
  `_catch_value` returning `None` for "knob hasn't caught yet."
- **Closure / factory function** — an inner function that "remembers" a variable
  from the outer function that built it; used to stamp out N handlers that differ
  by one value.
- **Reflection** — accessing attributes by string name at runtime (`getattr`,
  `setattr`), which is what lets one loop drive 19 buffers or 19 widgets.
- **Serialisation** — turning objects into bytes/text for storage (`json.dump`,
  `struct.pack`) and back.
- **Binding table** — a single map from names to widgets/fields that several
  behaviors all derive from, instead of parallel hand-maintained lists.
- **Catch / takeover** — refusing to let a physical control drive a value until its
  position matches the stored one, so nothing jumps.
- **Interlock** — a condition that must hold before a dangerous action is allowed
  (throttle at idle before arming).
- **Watchdog** — a timer that fires a safe action if something stops happening.
- **Dirty-checking** — skipping a write when the value hasn't changed.
- **Graceful degradation** — losing one capability instead of crashing.
- **Persistence** — saving state to disk so it survives the program exiting; here,
  notes as text and settings/gains as JSON.

---

*Suggested way to study: open the file, put your cursor on `_control_loop`
(L1230), and trace each branch outward — every path eventually touches one of the
four thread-crossing mechanisms. Once those four feel obvious, read
`_setup_bindings` (L2158) and ask of any other cluster of code: "is this fact
written down more than once?" Those two habits — knowing which thread owns what,
and noticing duplicated truth — are most of what separates code that survives
being edited from code that doesn't.*
