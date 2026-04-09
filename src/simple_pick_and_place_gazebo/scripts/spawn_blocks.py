
#!/usr/bin/env python3

import subprocess

MODELS_DIR = '/home/rinho/ros2_ws/src/simple_pick_and_place_gazebo/models'

blocks = [

    {'color': 'red',    'name': 'red_cuboid_1',    'x':  0.40, 'y':  0.00},

    {'color': 'blue',   'name': 'blue_cuboid_1',   'x':  0.00, 'y':  0.40},

    {'color': 'green',  'name': 'green_cuboid_1',  'x':  0.45, 'y': -0.10},

    {'color': 'yellow', 'name': 'yellow_cuboid_1', 'x':  0.10, 'y': -0.50},

]

for b in blocks:

    sdf = f'{MODELS_DIR}/{b["color"]}_cuboid/model.sdf'

    req = f'sdf_filename: "{sdf}" name: "{b["name"]}" pose: {{position: {{x: {b["x"]}, y: {b["y"]}, z: 0.05}}}}'

    cmd = ['gz', 'service', '-s', '/world/pick_and_place/create',

           '--reqtype', 'gz.msgs.EntityFactory',

           '--reptype', 'gz.msgs.Boolean',

           '--timeout', '1000', '--req', req]

    r = subprocess.run(cmd, capture_output=True, text=True)

    print(f'Spawned {b["name"]}: {r.stdout.strip()}')

