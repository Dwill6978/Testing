
import cflib.crtp, time
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

cflib.crtp.init_drivers()

def on_pkt(pk):
    print(f"port={pk.port:#x} ch={pk.channel} len={len(pk.data)} data[0]={pk.data[0] if pk.data else '?'} raw={bytes(pk.data).hex()}")

with SyncCrazyflie('radio://0/80/2M/E7E7E7E7E7') as scf:
    time.sleep(2)
    scf.cf.add_port_callback(0x06, on_pkt)
    scf.cf.param.set_value('locSrv.enA2AStream', '1')
    print("enA2AStream:", scf.cf.param.get_value('locSrv.enA2AStream'))
    print("Listening on port 0x06 (LOCALIZATION) — all channels/types...")
    time.sleep(30)

