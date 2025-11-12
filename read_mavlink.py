from pymavlink import mavutil

connection = mavutil.mavlink_connection('udp:0.0.0.0:14550')
print("🔄 Waiting for heartbeat...")
connection.wait_heartbeat()
print("✅ Heartbeat received. Requesting data streams...")

# Ask the Pixhawk to start sending position, attitude, and system status data
connection.mav.request_data_stream_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    4,  # rate in Hz
    1   # start sending
)

while True:
    msg = connection.recv_match(blocking=True)
    if not msg:
        continue

    msg_type = msg.get_type()

    if msg_type == "GLOBAL_POSITION_INT":
        print(
            f"🌍 Lat: {msg.lat/1e7:.6f}, Lon: {msg.lon/1e7:.6f}, Alt: {msg.alt/1000:.2f} m")

    elif msg_type == "ATTITUDE":
        print(
            f"🌀 Roll: {msg.roll:.2f}, Pitch: {msg.pitch:.2f}, Yaw: {msg.yaw:.2f}")

    elif msg_type == "SYS_STATUS":
        print(
            f"🔋 Battery: {msg.voltage_battery/1000:.2f} V, {msg.current_battery/100.0:.2f} A")

    elif msg_type == "HEARTBEAT":
        print(
            f"❤️ Heartbeat from system {getattr(msg, 'get_srcSystem', lambda: 'Unknown')()}")
