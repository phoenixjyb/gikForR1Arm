import numpy as np
from xml.etree import ElementTree as ET

def parse_origin_to_matrix(xyz_str, rpy_str):
    xyz = np.fromstring(xyz_str, sep=' ')
    rpy = np.fromstring(rpy_str, sep=' ')
    cx, cy, cz = np.cos(rpy)
    sx, sy, sz = np.sin(rpy)
    R = np.array([
        [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
        [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
        [-sy,     cy * sx,                cy * cx]
    ])
    T = np.eye(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = xyz
    return T

def compute_chain_transform(joint_map, chain):
    T = np.eye(4)
    for parent, child in chain:
        T = T @ joint_map[(parent, child)]
    return T

def extract_arm(root, prefix, root_link_name, new_root_name, remove_fingers=False):
    all_links = {link.attrib['name']: link for link in root.findall('link')}
    all_joints = {joint.attrib['name']: joint for joint in root.findall('joint')}
    arm_links = set()
    arm_joints = set()
    to_visit = [root_link_name]

    while to_visit:
        current = to_visit.pop()
        arm_links.add(current)
        for joint in root.findall('joint'):
            parent = joint.find('parent').attrib['link']
            child = joint.find('child').attrib['link']
            if parent == current and (prefix in child or (not remove_fingers and 'gripper' in child)):
                arm_joints.add(joint.attrib['name'])
                to_visit.append(child)

    for joint in root.findall('joint'):
        parent = joint.find('parent').attrib['link']
        child = joint.find('child').attrib['link']
        if child == root_link_name:
            arm_joints.add(joint.attrib['name'])
            arm_links.add(parent)

    link_elements = [all_links[n] for n in arm_links if n in all_links]
    joint_elements = [all_joints[n] for n in arm_joints if n in all_joints]

    if remove_fingers:
        joint_elements = [j for j in joint_elements if 'finger' not in j.find('child').attrib['link']]
        link_elements = [l for l in link_elements if 'finger' not in l.attrib['name']]

    new_root = ET.Element('link', name=new_root_name)
    inertial = ET.SubElement(new_root, 'inertial')
    ET.SubElement(inertial, 'origin', xyz="0 0 0", rpy="0 0 0")
    ET.SubElement(inertial, 'mass', value="0.001")
    ET.SubElement(inertial, 'inertia', ixx="1e-6", ixy="0", ixz="0", iyy="1e-6", iyz="0", izz="1e-6")
    link_elements.append(new_root)

    for joint in joint_elements:
        if joint.find('child').attrib['link'] == root_link_name:
            joint.find('parent').attrib['link'] = new_root_name
            break

    return link_elements, joint_elements

def save_robot(link_elements, joint_elements, filename):
    robot = ET.Element('robot', name=filename.split('.')[0])
    for link in link_elements:
        robot.append(link)
    for joint in joint_elements:
        robot.append(joint)
    ET.ElementTree(robot).write(filename, encoding='utf-8', xml_declaration=True)

def main():
    full_tree = ET.parse("fineUrdfs/r1_v2_1_0.urdf")
    root = full_tree.getroot()

    joints = {}
    for joint in root.findall('joint'):
        parent = joint.find('parent').attrib['link']
        child = joint.find('child').attrib['link']
        origin = joint.find('origin')
        xyz = origin.attrib.get('xyz', '0 0 0')
        rpy = origin.attrib.get('rpy', '0 0 0')
        joints[(parent, child)] = parse_origin_to_matrix(xyz, rpy)

    left_chain = [
        ("base_link", "torso_link1"),
        ("torso_link1", "torso_link2"),
        ("torso_link2", "torso_link3"),
        ("torso_link3", "torso_link4"),
        ("torso_link4", "left_arm_base_link")
    ]
    T_left = compute_chain_transform(joints, left_chain)

    right_chain = [
        ("base_link", "torso_link1"),
        ("torso_link1", "torso_link2"),
        ("torso_link2", "torso_link3"),
        ("torso_link3", "torso_link4"),
        ("torso_link4", "right_arm_base_link")
    ]
    T_right = compute_chain_transform(joints, right_chain)

    def mat_to_xyzrpy(T):
        xyz = T[0:3, 3]
        R = T[0:3, 0:3]
        sy = -R[2, 0]
        cy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
        rpy = np.arctan2([R[2,1], R[2,2], sy, R[1,0], R[0,0]], [cy]*5)
        return ' '.join(map(str, xyz)), ' '.join(map(str, [rpy[2], rpy[0], rpy[3]]))

    left_links, left_joints = extract_arm(root, "left_arm", "left_arm_base_link", "left_arm_root")
    xyz, rpy = mat_to_xyzrpy(T_left)
    for j in left_joints:
        if j.find('child').attrib['link'] == 'left_arm_base_link':
            j.find('origin').attrib['xyz'] = xyz
            j.find('origin').attrib['rpy'] = rpy
            break

    right_links, right_joints = extract_arm(root, "right_arm", "right_arm_base_link", "right_arm_root", remove_fingers=True)
    xyz, rpy = mat_to_xyzrpy(T_right)
    for j in right_joints:
        if j.find('child').attrib['link'] == 'right_arm_base_link':
            j.find('origin').attrib['xyz'] = xyz
            j.find('origin').attrib['rpy'] = rpy
            break

    save_robot(left_links, left_joints, "left_arm_aligned.urdf")
    save_robot(right_links, right_joints, "right_arm_aligned.urdf")

    merged = ET.Element('robot', name='dual_arm_merged')
    all_links = {l.attrib['name']: l for l in left_links + right_links}
    all_joints = {j.attrib['name']: j for j in left_joints + right_joints}
    for l in all_links.values():
        merged.append(l)
    for j in all_joints.values():
        merged.append(j)
    ET.ElementTree(merged).write("dual_arm_merged.urdf", encoding='utf-8', xml_declaration=True)

if __name__ == "__main__":
    main()
