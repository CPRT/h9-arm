from geometry_msgs.msg import Point
import math

ARM_BASE_LENGTH = 0.30 # m, pivot to pivot
ARM_UPPER_LENGTH = 0.30 # m, pivot to end-effector axis

def validate_point(pose: Point) -> bool:
  return math.hypot(pose.x, pose.y, pose.z) < ARM_UPPER_LENGTH + ARM_BASE_LENGTH

def calculate_joint_state(pose: Point):
  if not validate_point(pose):
    raise ValueError("Invalid target point")
  print("Target Point:",pose.x, pose.y, pose.z)
  # Identify base/turrent rotation:
  base_angle = math.degrees(math.atan2(pose.y, pose.x))
  print("Base Angle:",base_angle)

  # Identify elevation angle of lower arm
  vector_length = math.hypot(pose.x, pose.y, pose.z) # length of pose vector
  shoulder_angle_pt1 = math.acos((ARM_UPPER_LENGTH**2 - vector_length**2 - ARM_BASE_LENGTH**2)/(-2.0 * ARM_BASE_LENGTH * vector_length)) #cosine law
  shoulder_angle_pt2 = math.atan2(pose.z, math.sqrt(pose.x**2+pose.y**2))
  shoulder_angle = shoulder_angle_pt1 + shoulder_angle_pt2
  print("Shoulder Angle:",math.degrees(shoulder_angle))

  # Identify elevation angle of upper arm
  elbow_angle = math.degrees(math.acos((vector_length**2 - ARM_UPPER_LENGTH**2 - ARM_BASE_LENGTH**2)/(-2.0 * ARM_BASE_LENGTH * ARM_UPPER_LENGTH))) #cosine law
  print("Elbow Angle:", elbow_angle)


  # Calculate arm vectors
  xy_length = (ARM_BASE_LENGTH*math.sin(math.pi/2-shoulder_angle))/math.sin(math.pi/2) # length of lower arm vector projected onto ground plane

  # Lower arm:
  lower_z_comp = (ARM_BASE_LENGTH*math.sin(shoulder_angle))/math.sin(math.pi/2)
  lower_y_comp = (xy_length*math.sin(base_angle))/math.sin(math.pi/2)
  lower_x_comp = (xy_length*math.sin(math.pi/2-base_angle))/math.sin(math.pi/2)
  print("Lower Arm Vector:", lower_x_comp, lower_y_comp, lower_z_comp)

  # Upper arm:
  upper_z_comp = pose.z - lower_z_comp
  upper_y_comp = pose.y - lower_y_comp
  upper_x_comp = pose.x - lower_x_comp
  print("Upper arm Vector:",upper_x_comp, upper_y_comp, upper_z_comp)


calculate_joint_state(Point(0.3,0.3,0.3))
calculate_joint_state(Point(-0.3,0.3,0.3))
calculate_joint_state(Point(0.3,-0.3,0.3))
calculate_joint_state(Point(-0.3,-0.3,0.3))
calculate_joint_state(Point(0.01,0.01,0.3))