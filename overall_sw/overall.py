# general remarks:
# the absolute frame of reference is determined by px4
# the relative frame of reference is fixed to the drone
# apart from that, we have frames of references for the cameras
# we need to pay attention to properly convert between these
# the z coordinate is the same for all frames of reference
# therefore, a pose can be described by x, y, z and yaw


# remaining todos
# use the position cache in the other functions
# implement actuator control and rotation sensor reading, see also https://gitlab.com/voxl-public/voxl-sdk/core-libs/libapq8096-io/-/tree/master/lib/python and voxl io in general
# adoption of some functions to the simulation
# ...


import asyncio

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityBodyYawspeed

# these functions should be implmented in the other software tasks
from modules.cone_detection import detect_cone
#from modules.window_detection import detect_window
from modules.lane_detection import detect_lane_start, detect_lane_end, detect_lane_start_reverse, detect_lane_end_reverse

from math import atan2,pi

ORIENT = 1
FLY_TO_WINDOW = 2
FLY_TO_PICKUP_ZONE = 3
PREPARE_PICKUP = 4
PICKUP = 5
ORIENT_BACK = 6
FLY_TO_WINDOW_BACK = 7
FLY_TO_DROP_ZONE = 8
PREPARE_DROP = 9
DROP = 10

current_state = START

# this is the height we fly to before pickup
# we need to find a value where the gripper is not touching the cone but is close to it
GRIP_HEIGHT_BEFORE_PICKUP = 0.5 

# this is the distance from the position measurement to center of the gripper in the x direction
GRIPPER_OFFSET_X = 0.1 

# this is the distance from the position measurement to center of the gripper in the y direction
GRIPPER_OFFSET_Y = 0.1

# it might be useful to keep track of previously reached points
# in this dict we keep track of the positions when a given state transition happened
position_cache = {i:[] for i in range(11)}

# gripper functions:

def read_gripper_rotation_1():
    pass

def read_gripper_rotation_2():
    pass

def read_gripper_rotation_3():
    pass

def read_gripper_rotation_4():
    pass

def read_gripper_rotations():
    return [read_gripper_rotation_1(), read_gripper_rotation_2(),
             read_gripper_rotation_3(), read_gripper_rotation_4()]

def release():
    pass

# for a lot of the following functions, we could use voxl information (vio) instead of px4 information
async def get_position(drone):
    async for position in drone.telemetry.position_velocity_ned():
        return position.position

async def reached_absolute_setpoint(current_setpoint, drone):
    tolerance = 0.1
    async for position in drone.telemetry.position_velocity_ned():
        if abs(position.position.north_m - current_setpoint.north_m) < tolerance and \
            abs(position.position.east_m - current_setpoint.east_m) < tolerance and \
            abs(position.position.down_m - current_setpoint.down_m) < tolerance and\
                abs(position.yaw_deg - current_setpoint.yaw_deg) < tolerance:
            return True
    return False

async def get_bottom_camera_image(drone):
    pass

async def get_front_camera_image(drone):
    pass

async def get_height(drone):
    async for position in drone.telemetry.position_velocity_ned():
        return position.position.down_m

async def fly_to_absolute_setpoint(setpoint, drone):
    await drone.offboard.set_position_ned(setpoint)
    while not await reached_absolute_setpoint(setpoint, drone):
        await asyncio.sleep(0.1)

async def fly_to_relative_setpoint(setpoint, drone):
    drone_pos = await get_position(drone)
    await fly_to_absolute_setpoint(PositionNedYaw(drone_pos.north_m + setpoint.north_m, drone_pos.east_m + setpoint.east_m, drone_pos.down_m + setpoint.down_m, drone_pos.yaw_deg + setpoint.yaw_deg), drone)


async def fly_in_absolute_direction(yaw, velocity, drone):
    pass

async def fly_in_relative_direction(yaw, velocity , drone):
    pass

async def fly_forwards(velocity, drone):
    drone.offboard.set_velocity_body(VelocityBodyYawspeed(velocity, 0.0, 0.0, 0.0))

# todo: implement correctly
def bottom_camera_to_relative(setpoint):
    return setpoint

# might be implmented differently for simulation
async def descend_until_gripped():
    max_gripper_rotations = read_gripper_rotations()
    gripped = [False, False, False, False]
    threshold = 0.1 # again, we need to find a good value
    while True:
        gripper_rotations = read_gripper_rotations()
        max_gripper_rotations = [max(gripper_rotations[i], max_gripper_rotations[i]) for i in range(4)]
        rot_diffs = [max_gripper_rotations[i] - gripper_rotations[i] for i in range(4)]
        gripped = [gripped[i] or rot_diffs[i] > threshold for i in range(4)]
        if all(gripped):
            break
        await asyncio.sleep(0.1)
# functions for each state

async def orient(drone):
    # this way to use the position cache still needs to be implmented in the other functions
    if not len(position_cache[ORIENT]) == 0:
        await fly_to_absolute_setpoint(position_cache[ORIENT][-1], drone)
    lane_start_x,lane_start_y, lane_direction = detect_lane_start()

    if lane_start_x is None:
        # in this case some kind of exploration or previous knowledge about the lane start
        #  could be used
        raise Exception("lane start not found")
    else:
        camera_setpoint = PositionNedYaw(lane_start_x, lane_start_y,lane_direction, 0.0)
        relative_setpoint = bottom_camera_to_relative(camera_setpoint)
        await fly_to_relative_setpoint(relative_setpoint, drone)
        # now we could double check




#can be implemented later, when the simpler mission is solved
async def fly_to_window(drone):
    pass

async def fly_to_pickup_zone(drone):
    forward_velocity = 1 # a good value needs to be found
    await fly_forwards(forward_velocity, drone)
    # a better break condition is probably still needed
    # loop could also include periodic correction of the flight direction
    while True:
        lane_end = detect_lane_end(get_bottom_camera_image(drone))
        if lane_end is not None:
            camera_setpoint = PositionNedYaw(lane_end[0], lane_end[1], await get_height(drone), 0.0)
            setpoint = bottom_camera_to_relative(camera_setpoint)
            await fly_to_relative_setpoint(setpoint, drone)
            break
        await asyncio.sleep(0.1)

# pickup and prepare_pickup currently assume the gripper design jerry suggested
# also assume that the cone can be detected from the lane end position
async def prepare_pickup(drone):
    x_cone, y_cone = detect_cone(get_bottom_camera_image(drone))
    if x_cone is None:
        # in this case some kind of exploration or previous knowledge about the cone
        #  could be used
        raise Exception("Cone not found")
    camera_setpoint = PositionNedYaw(x_cone, y_cone, GRIP_HEIGHT_BEFORE_PICKUP, 0.0)
    relative_setpoint = bottom_camera_to_relative(camera_setpoint)
    await fly_to_relative_setpoint(relative_setpoint, drone)
    

async def pickup(drone):
    descent_velocity = 1 # a good value needs to be found
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, descent_velocity, 0.0))
    await descend_until_gripped()
    ascension_height = 1 # a good value needs to be found
    setpoint = PositionNedYaw(0,0, ascension_height, 0.0)
    await fly_to_relative_setpoint(setpoint, drone)

async def orient_back(drone):
    lane_end = position_cache[PREPARE_PICKUP]
    await fly_to_absolute_setpoint(lane_end, drone)
    lane_start_x,lane_start_y, lane_direction = detect_lane_start_reverse()

    if lane_start_x is None:
        # in this case some kind of exploration or previous knowledge about the lane start
        #  could be used
        raise Exception("lane start not found")
    else:
        camera_setpoint = PositionNedYaw(lane_start_x, lane_start_y,lane_direction, 0.0)
        relative_setpoint = bottom_camera_to_relative(camera_setpoint)
        await fly_to_relative_setpoint(relative_setpoint, drone)

#similar to fly_to_window
async def fly_to_window_back(drone):
    pass

async def fly_to_drop_zone(drone):
    forward_velocity = 1 # a good value needs to be found
    await fly_forwards(forward_velocity, drone)
    # a better break condition is probably still needed
    # loop could also include periodic correction of the flight direction
    while True:
        lane_end = detect_lane_end(get_bottom_camera_image(drone))
        if lane_end is not None:
            camera_setpoint = PositionNedYaw(lane_end[0], lane_end[1], await get_height(drone), 0.0)
            setpoint = bottom_camera_to_relative(camera_setpoint)
            await fly_to_relative_setpoint(setpoint, drone)
            break
        await asyncio.sleep(0.1)

async def prepare_drop(drone):
    x_tower, y_tower, z_tower = detect_tower(get_bottom_camera_image(drone))
    height_above_tower = 1 # a good value needs to be found
    if x_tower is None:
        # in this case some kind of exploration or previous knowledge about the cone
        #  could be used
        raise Exception("Tower not found")
    camera_setpoint = PositionNedYaw(x_cone, y_cone, z_cone + height_above_tower, 0.0)
    relative_setpoint = bottom_camera_to_relative(camera_setpoint)
    await fly_to_relative_setpoint(relative_setpoint, drone)

async def drop(drone):
    release()

state_to_function = {ORIENT: orient, FLY_TO_WINDOW: fly_to_window, FLY_TO_PICKUP_ZONE: fly_to_pickup_zone,
                      PREPARE_PICKUP: prepare_pickup, PICKUP: pickup, ORIENT_BACK: orient_back, 
                      FLY_TO_WINDOW_BACK: fly_to_window_back, FLY_TO_DROP_ZONE: fly_to_drop_zone,
                        PREPARE_DROP: prepare_drop, DROP: drop}	

state_to_next_state = {ORIENT: FLY_TO_WINDOW, FLY_TO_WINDOW: FLY_TO_PICKUP_ZONE, FLY_TO_PICKUP_ZONE: PREPARE_PICKUP,
                        PREPARE_PICKUP: PICKUP, PICKUP: ORIENT_BACK, ORIENT_BACK: FLY_TO_WINDOW_BACK,
                        FLY_TO_WINDOW_BACK: FLY_TO_DROP_ZONE, FLY_TO_DROP_ZONE: PREPARE_DROP, PREPARE_DROP: DROP,
                        DROP: ORIENT}

async def run_fsm(drone):
    while True:
        position_cache[current_state].append(await get_position(drone))
        await state_to_function[current_state](drone)
        current_state = state_to_next_state[current_state]
        
async def check_for_landing(drone):
    pass

# we will probably need different actions depending on the state, e.g. releasing the cone when carrying it
async def land(drone):
    await drone.action.land()

async def main():
    # copypaste from https://github.com/mavlink/MAVSDK-Python/blob/main/examples/offboard_position_ned.py
    # the prints can be removed
    drone = System()
    await drone.connect(system_address="udp://:14540")
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered with UUID: {state.uuid}")
            break
    
    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok:
            print("Global position estimate ok")
            break
    
    print("-- Arming")
    await drone.action.arm()

    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed \
                with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return
    
    # end of copypaste
    initial_height = 1.5
    current_setpoint = PositionNedYaw(0.0, 0.0, -initial_height, 0.0)
    await drone.offboard.set_position_ned(current_setpoint)

    while not await reached_setpoint(current_setpoint, drone):
        await asyncio.sleep(0.1)
    
    # now the plan is to run the state machine and land the drone when we send the instruction to do so
    fsm_task = asyncio.create_task(run_fsm(drone))
    while not check_for_landing(drone):
        await asyncio.sleep(0.1)
    
    fsm_task.cancel()
    await land(drone)

if __name__ == "__main__":
    asyncio.run(main())

    
    






