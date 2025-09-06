import os, time, math, csv, datetime
import carla

def norm(v): return math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)

def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(5.0)
    world = client.get_world()
    carla_map = world.get_map()

    # Find the player vehicle spawned by manual_control (role_name="hero")
    actors = world.get_actors().filter("vehicle.*")
    hero = None
    for a in actors:
        if a.attributes.get("role_name") == "hero":
            hero = a
            break
    if hero is None:
        raise RuntimeError("No vehicle with role_name=hero found. Run manual_control.py first.")

    os.makedirs("logs", exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = os.path.join("logs", f"session_{stamp}.csv")

    fields = [
        "t","x","y","z","yaw_deg","speed_mps","speed_kph",
        "throttle","brake","steer",
        "lane_id","lane_type","lane_offset_m",
        "speed_limit_kph","traffic_light_state"
    ]
    with open(out_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()

        print(f"[Logger] Writing to {out_path} (10 Hz). Ctrl+C to stop.")
        t0 = time.time()
        dt = 0.1  # 10 Hz
        try:
            while True:
                now = time.time()
                # Get basic kinematics
                transform = hero.get_transform()
                loc, rot = transform.location, transform.rotation
                vel = hero.get_velocity()
                speed_mps = norm(vel)
                speed_kph = 3.6 * speed_mps

                # Control inputs (works even if controlled by manual_control)
                control = hero.get_control()

                # Waypoint & lane info
                wp = carla_map.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
                lane_id = wp.lane_id if wp else None
                lane_type = str(wp.lane_type) if wp else None

                # Lateral offset from lane center (approx.)
                lane_tf = wp.transform if wp else transform
                fwd = lane_tf.get_forward_vector()
                right = carla.Vector3D(fwd.y, -fwd.x, 0.0)  # 2D right vector approx
                center = lane_tf.location
                diff = carla.Vector3D(loc.x - center.x, loc.y - center.y, 0.0)
                lane_offset = (diff.x * right.x + diff.y * right.y) / max(1e-6, math.sqrt(right.x**2 + right.y**2))

                # Speed limit & traffic light
                speed_limit_kph = hero.get_speed_limit()
                tls = hero.get_traffic_light()
                tl_state = str(tls.get_state()) if tls else "Unknown"

                writer.writerow({
                    "t": now - t0,
                    "x": loc.x, "y": loc.y, "z": loc.z,
                    "yaw_deg": rot.yaw,
                    "speed_mps": round(speed_mps, 3),
                    "speed_kph": round(speed_kph, 2),
                    "throttle": round(control.throttle, 3),
                    "brake": round(control.brake, 3),
                    "steer": round(control.steer, 3),
                    "lane_id": lane_id,
                    "lane_type": lane_type,
                    "lane_offset_m": round(lane_offset, 3),
                    "speed_limit_kph": speed_limit_kph,
                    "traffic_light_state": tl_state
                })

                # Sleep to maintain ~10 Hz
                time_to_next = dt - ((time.time() - now) % dt)
                time.sleep(max(0.0, time_to_next))
        except KeyboardInterrupt:
            print("\n[Logger] Stopped.")

if __name__ == "__main__":
    main()
