import sim
import pybullet as p
import numpy as np

MAX_ITERS = 10000
delta_q = 1 #1

def visualize_path(q_1, q_2, env, color=[0, 1, 0]):
    """
    Draw a line between two positions corresponding to the input configurations
    :param q_1: configuration 1
    :param q_2: configuration 2
    :param env: environment
    :param color: color of the line, please leave unchanged.
    """
    # obtain position of first point
    env.set_joint_positions(q_1)
    point_1 = p.getLinkState(env.robot_body_id, 9)[0]
    # obtain position of second point
    env.set_joint_positions(q_2)
    point_2 = p.getLinkState(env.robot_body_id, 9)[0]
    # draw line between points
    p.addUserDebugLine(point_1, point_2, color, 1.0)

def rrt(q_init, q_goal, MAX_ITERS, delta_q, steer_goal_p, env):
    """
    :param q_init: initial configuration
    :param q_goal: goal configuration
    :param MAX_ITERS: max number of iterations
    :param delta_q: steer distance
    :param steer_goal_p: probability of steering towards the goal
    :returns path: list of configurations (joint angles) if found a path within MAX_ITERS, else None
    """
    # ========= TODO: Problem 3 ========
    # Implement RRT code here. This function should return a list of joint configurations
    # that the robot should take in order to reach q_goal starting from q_init
    # Use visualize_path() to visualize the edges in the exploration tree for part (b)
    V = set()
    E = set()
    q_init = tuple(q_init)
    V.add(q_init)

    print("q_goal", q_goal)

    for i in range(MAX_ITERS):
        q_rand = SemiRandomSample(steer_goal_p, q_goal)
        q_nearest = Nearest(q_rand, V)
        q_new = Steer(q_nearest, q_rand, delta_q)
        if env.check_collision(q_new) == False:
            q_new = tuple(q_new)
            V.add(q_new)
            E.add((q_nearest, q_new))
            visualize_path(q_nearest, q_new, env)
            if Distance(q_new, q_goal) < delta_q:
                q_goal = tuple(q_goal)
                V.add(q_goal)
                E.add((q_new, q_goal))
                print("GOAL REACHED")
                return Path(q_init, q_goal, E, V)
    

    # ==================================
    return None

def execute_path(path_conf, env):
    # ========= TODO: Problem 3 ========
    # 1. Execute the path while visualizing the location of joint 5 
    #    (see Figure 2 in homework manual)
    #    You can get the position of joint 5 with:
    #         p.getLinkState(env.robot_body_id, 9)[0]
    #    To visualize the position, you should use sim.SphereMarker
    #    (Hint: declare a list to store the markers)
    # 2. Drop the object (Hint: open gripper, step the simulation, close gripper)
    # 3. Return the robot to original location by retracing the path 
    spheres = []
    for joint_states in path_conf:
        env.move_joints(joint_states)
        pos_joint_5 = p.getLinkState(env.robot_body_id, 9)[0]
        newsphere = sim.SphereMarker(pos_joint_5)
        spheres.append(newsphere)
    env.open_gripper()
    #env.step_simulation()
    env.close_gripper()

    for joint_states in reversed(path_conf):
        env.move_joints(joint_states)
        pos_joint_5 = p.getLinkState(env.robot_body_id, 9)[0]
        sim.SphereMarker(pos_joint_5)

    # ==================================
    return None

def SemiRandomSample(steer_goal_p, q_goal):
    random_point = np.random.uniform(low=-np.pi, high=np.pi, size=(6,))
    random_point = tuple(random_point)
    choices = [1, 2]
    prob = [1 - steer_goal_p, steer_goal_p]
    res = np.random.choice(choices, p=prob)
    if res == 1:
        return random_point
    else:
        return q_goal

# is this in radians or degrees?
def Distance(q1, q2):
    dist = 0
    for i in range(6):
        dist += np.abs(q1[i] - q2[i])
    return dist

def Nearest(q_rand, V):
    mindist = 100000000
    minq = None
    for q in V:
        if Distance(q, q_rand) < mindist:
            mindist = Distance(q, q_rand)
            minq = q
    return minq

def Steer(q_nearest, q_rand, delta_q):
    q_new = np.zeros(6)
    dist = Distance(q_rand, q_nearest)
    for i in range(6):
        q_new[i] = q_nearest[i] + (delta_q/dist) * (q_rand[i] - q_nearest[i])
    return q_new

def Path(q_init, q_goal, E, V):
    # find path from q_init to q_goal
    # return list of configurations
    path = []
    path.append(q_goal)
    q = q_goal
    while q != q_init:
        for edge in E:
            if edge[1] == q:
                q = edge[0]
                path.append(q)
                break
    path.reverse()
    return path