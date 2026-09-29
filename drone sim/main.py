"""Drone Flight Training Simulator
Python + Ursina (Panda3D). Run:  python main.py
"""
import math
import random

from ursina import *

# ----------------------------------------------------------------- constants
G = 9.81                 # gravity (m/s^2)
DRAG = 0.5               # linear air drag (1/s)
MAX_TILT = 30            # max pitch/roll (degrees)
TILT_RESPONSE = 6.0      # how fast attitude follows input
YAW_RATE = 100           # deg/s
THROTTLE_RATE = 0.6      # throttle change per second
REST_Y = 0.15            # body height when sitting on the ground
CRASH_SPEED = 4.5        # m/s descent rate that counts as a crash
CRASH_TILT = 15          # degrees of tilt at touchdown that counts as a crash
PAD_RADIUS = 4.0
RING_RADIUS = 3.0
NUM_BUILDINGS = 30
NUM_RINGS = 8
CAM_MODES = ['CHASE', 'FPV', 'ORBIT']

app = Ursina(title='Drone Flight Training Simulator', vsync=True)
window.exit_button.visible = False
window.fps_counter.enabled = True

random.seed(7)

# --------------------------------------------------------------------- world
Sky()
DirectionalLight(rotation=(50, -30, 0))
AmbientLight(color=color.rgba(120, 120, 130, 255))
Entity(model='plane', scale=600, color=color.rgb(70, 120, 60), texture='white_cube',
       texture_scale=(120, 120))

PADS = [
    {'name': 'HOME', 'pos': Vec3(0, 0, 0), 'color': color.azure},
    {'name': 'PAD B', 'pos': Vec3(70, 0, 60), 'color': color.magenta},
]
for pad in PADS:
    Entity(model='cube', position=pad['pos'] + Vec3(0, 0.03, 0),
           scale=(PAD_RADIUS * 2, 0.06, PAD_RADIUS * 2), color=pad['color'])
    Entity(model='cube', position=pad['pos'] + Vec3(0, 7, 0),
           scale=(0.3, 14, 0.3), color=pad['color'])  # beacon pillar

buildings = []  # (cx, cz, half_w, half_d, height)


def near_pad(x, z, margin):
    return any(math.hypot(x - p['pos'].x, z - p['pos'].z) < margin for p in PADS)


tries = 0
while len(buildings) < NUM_BUILDINGS and tries < 5000:
    tries += 1
    x, z = random.uniform(-160, 160), random.uniform(-160, 160)
    w, d, h = random.uniform(6, 14), random.uniform(6, 14), random.uniform(8, 40)
    if near_pad(x, z, 22):
        continue
    if any(abs(x - b[0]) < w / 2 + b[2] + 8 and abs(z - b[1]) < d / 2 + b[3] + 8 for b in buildings):
        continue
    buildings.append((x, z, w / 2, d / 2, h))
    Entity(model='cube', position=(x, h / 2, z), scale=(w, h, d),
           color=color.hsv(0, 0, random.uniform(0.35, 0.75)), texture='white_cube',
           texture_scale=(w / 3, h / 3))


def clear_of_buildings(x, z, margin):
    return not any(abs(x - b[0]) < b[2] + margin and abs(z - b[1]) < b[3] + margin for b in buildings)


class Ring:
    def __init__(self):
        while True:
            self.x, self.z = random.uniform(-120, 120), random.uniform(-120, 120)
            if clear_of_buildings(self.x, self.z, 8) and not near_pad(self.x, self.z, 10):
                break
        self.y = random.uniform(5, 18)
        self.yaw = random.uniform(0, 360)
        self.done = False
        self.prev_lz = None
        self.root = Entity(position=(self.x, self.y, self.z), rotation_y=self.yaw)
        self.parts = []
        for i in range(16):
            a = i * math.tau / 16
            self.parts.append(Entity(parent=self.root, model='cube', scale=0.55, color=color.orange,
                                     position=(math.cos(a) * RING_RADIUS, math.sin(a) * RING_RADIUS, 0)))

    def set_done(self, done):
        self.done = done
        for p in self.parts:
            p.color = color.lime if done else color.orange

    def local(self, pos):
        """Drone position in the ring's local frame (z = along ring normal)."""
        dx, dy, dz = pos.x - self.x, pos.y - self.y, pos.z - self.z
        a = math.radians(self.yaw)
        return dx * math.cos(a) - dz * math.sin(a), dy, dx * math.sin(a) + dz * math.cos(a)


rings = [Ring() for _ in range(NUM_RINGS)]

# --------------------------------------------------------------------- drone
drone = Entity()
Entity(parent=drone, model='cube', scale=(0.5, 0.15, 0.5), color=color.dark_gray)
Entity(parent=drone, model='cube', scale=(0.15, 0.1, 0.35), position=(0, 0.02, 0.3), color=color.red)  # nose
rotors = []
for sx in (-1, 1):
    for sz in (-1, 1):
        Entity(parent=drone, model='cube', scale=(0.7, 0.03, 0.05), position=(sx * 0.2, 0.05, sz * 0.2),
               rotation_y=45 * sx * sz, color=color.black)
        rotors.append(Entity(parent=drone, model='cube', scale=(0.45, 0.02, 0.06),
                             position=(sx * 0.35, 0.12, sz * 0.35), color=color.light_gray))


class State:
    pass


s = State()


def reset():
    s.pos = Vec3(0, REST_Y, 0)
    s.vel = Vec3(0, 0, 0)
    s.pitch = s.roll = 0.0
    s.yaw = 0.0
    s.throttle = 0.0
    s.crashed = False
    s.landed = True
    s.last_pad = None
    s.score = 0
    s.cam = 0
    s.orbit_t = 0.0
    for r in rings:
        r.set_done(False)
        r.prev_lz = None
    show_msg('Ready. Hold SPACE to take off.', color.white, 4)


# ----------------------------------------------------------------------- HUD
hud = Text(position=window.top_left + Vec2(0.02, -0.03), scale=1.3, color=color.white, background=True)
msg = Text(text='', origin=(0, 0), y=0.35, scale=2, color=color.white)
help_text = Text(origin=(0, 0), scale=1.1, background=True, enabled=True, text=(
    'CONTROLS\n'
    'SPACE / L-SHIFT  throttle up / down\n'
    'W / S            pitch forward / back\n'
    'A / D            roll left / right\n'
    'Q / E            yaw left / right\n'
    'R reset   C camera   H help   ESC quit\n\n'
    'Hover throttle is ~50%. Fly through orange rings,\n'
    'land gently on HOME / PAD B (alternate pads) for points.'))
msg_timer = 0.0


def show_msg(text, col=color.white, seconds=3):
    global msg_timer
    msg.text = text
    msg.color = col
    msg_timer = seconds


def crash(reason):
    s.crashed = True
    s.vel = Vec3(0, 0, 0)
    show_msg(f'CRASHED: {reason}\nPress R to reset', color.red, 999)


# ------------------------------------------------------------------- physics
def tilt_degrees():
    p, r = math.radians(s.pitch), math.radians(s.roll)
    v = Vec3(math.sin(r) * math.cos(p), math.cos(r) * math.cos(p), math.sin(p) * math.cos(r)).normalized()
    return math.degrees(math.acos(max(-1, min(1, v.y)))), v


def step_physics(dt):
    k = held_keys
    s.throttle = max(0.0, min(1.0, s.throttle + (k['space'] - k['left shift']) * THROTTLE_RATE * dt))
    target_pitch = (k['w'] - k['s']) * MAX_TILT
    target_roll = (k['d'] - k['a']) * MAX_TILT
    blend = min(1.0, TILT_RESPONSE * dt)
    s.pitch += (target_pitch - s.pitch) * blend
    s.roll += (target_roll - s.roll) * blend
    s.yaw += (k['e'] - k['q']) * YAW_RATE * dt

    tilt, v = tilt_degrees()
    y = math.radians(s.yaw)
    thrust = Vec3(v.x * math.cos(y) + v.z * math.sin(y), v.y, -v.x * math.sin(y) + v.z * math.cos(y))
    acc = thrust * (s.throttle * 2 * G) + Vec3(0, -G, 0) - s.vel * DRAG
    s.vel += acc * dt
    s.pos += s.vel * dt

    # ground contact
    if s.pos.y <= REST_Y:
        if not s.landed:
            if -s.vel.y > CRASH_SPEED:
                return crash('descended too fast')
            if tilt > CRASH_TILT:
                return crash('landed tilted')
            s.landed = True
            check_pad_landing()
        s.pos.y = REST_Y
        s.vel.y = max(0.0, s.vel.y)
        f = max(0.0, 1 - 8 * dt)
        s.vel.x *= f
        s.vel.z *= f
    elif s.pos.y > REST_Y + 0.4:
        s.landed = False

    # buildings
    for cx, cz, hw, hd, h in buildings:
        if abs(s.pos.x - cx) < hw + 0.4 and abs(s.pos.z - cz) < hd + 0.4 and s.pos.y < h + 0.2:
            return crash('hit a building')

    # rings
    for r in rings:
        lx, ly, lz = r.local(s.pos)
        if not r.done and r.prev_lz is not None and r.prev_lz * lz < 0 and math.hypot(lx, ly) < RING_RADIUS:
            r.set_done(True)
            s.score += 100
            show_msg('Ring! +100', color.orange, 1.5)
            if all(x.done for x in rings):
                s.score += 500
                show_msg('All rings cleared! +500', color.lime, 4)
        r.prev_lz = lz


def check_pad_landing():
    speed = math.hypot(s.vel.x, s.vel.z)
    for pad in PADS:
        d = math.hypot(s.pos.x - pad['pos'].x, s.pos.z - pad['pos'].z)
        if d < PAD_RADIUS and speed < 2.0 and s.last_pad != pad['name']:
            pts = 50 + round(100 * (1 - d / PAD_RADIUS))
            s.score += pts
            s.last_pad = pad['name']
            show_msg(f'{pad["name"]} landing +{pts}', color.azure, 2.5)
            return


# ------------------------------------------------------------------- cameras
def update_camera(dt):
    mode = CAM_MODES[s.cam]
    drone.visible = mode != 'FPV'
    camera.fov = 100 if mode == 'FPV' else 85
    y = math.radians(s.yaw)
    fwd = Vec3(math.sin(y), 0, math.cos(y))
    if mode == 'CHASE':
        target = s.pos - fwd * 9 + Vec3(0, 3.5, 0)
        camera.position = lerp(camera.position, target, min(1, 5 * dt))
        camera.position = Vec3(camera.x, max(0.6, camera.y), camera.z)
        camera.look_at(s.pos + Vec3(0, 0.5, 0))
    elif mode == 'FPV':
        camera.position = s.pos + fwd * 0.4 + Vec3(0, 0.15, 0)
        camera.rotation = Vec3(s.pitch, s.yaw, -s.roll)
    else:
        s.orbit_t += dt * 0.15
        camera.position = Vec3(s.pos.x + math.cos(s.orbit_t) * 70, 55, s.pos.z + math.sin(s.orbit_t) * 70)
        camera.look_at(s.pos)


# ---------------------------------------------------------------------- loop
def update():
    global msg_timer
    dt = min(time.dt, 0.05)
    if not s.crashed:
        step_physics(dt)

    drone.position = s.pos
    drone.rotation = Vec3(s.pitch, s.yaw, -s.roll)
    spin = 60 * (0.2 + s.throttle * 4) if not s.crashed else 0
    for i, r in enumerate(rotors):
        r.rotation_y += spin * (1 if i % 2 else -1) * dt * 60

    update_camera(dt)

    speed = s.vel.length()
    todo = [r for r in rings if not r.done]
    nxt = ''
    if todo:
        nearest = min(todo, key=lambda r: math.dist((r.x, r.y, r.z), tuple(s.pos)))
        nxt = f'\nNearest ring: {math.dist((nearest.x, nearest.y, nearest.z), tuple(s.pos)):.0f} m'
    hud.text = (f'ALT  {max(0, s.pos.y - REST_Y):5.1f} m\nSPD  {speed * 3.6:5.1f} km/h\n'
                f'THR  {s.throttle * 100:4.0f} %\nSCORE {s.score}\n'
                f'RINGS {sum(r.done for r in rings)}/{NUM_RINGS}  CAM {CAM_MODES[s.cam]}{nxt}')

    if msg_timer > 0:
        msg_timer -= dt
        if msg_timer <= 0:
            msg.text = ''


def input(key):
    if key == 'r':
        reset()
    elif key == 'c':
        s.cam = (s.cam + 1) % len(CAM_MODES)
    elif key == 'h':
        help_text.enabled = not help_text.enabled
    elif key == 'escape':
        application.quit()


reset()
app.run()
