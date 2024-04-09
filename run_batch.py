import os

def MakeDir(nd):
    if not os.path.exists(nd):
        os.mkdir(nd)

dataroot = "/media/data/datasets/oivio/selected_seq"
saving_root = "/media/code/ubuntu_files/airvo/experiments/vo_results/iros/oivio/"
workspace = "/media/code/ubuntu_files/airvo/ros2_ws"
traj_gt_dir = "/media/code/ubuntu_files/airvo/experiments/traj_gt/oivio"

MakeDir(saving_root)
traj_saving_root = os.path.join(saving_root, "traj")
MakeDir(traj_saving_root)
eva_root = os.path.join(saving_root, "evaluation")
MakeDir(eva_root)
eva_seq_root = os.path.join(saving_root, "seq")
MakeDir(eva_seq_root)
eva_sum_root = os.path.join(saving_root, "sum")
MakeDir(eva_sum_root)

sequences = os.listdir(dataroot)
for sequence in sequences:
    seq_dataroot = os.path.join(dataroot, sequence)
    seq_traj_file = sequence + ".txt"
    seq_traj_path = os.path.join(traj_saving_root, seq_traj_file)
    if os.path.exists(seq_traj_path):
        continue

    # Launch the ROS 2 node with the appropriate parameters
    os.system(f"cd {workspace} && ros2 launch air_vo oivio.launch.py dataroot:={seq_dataroot} traj_path:={seq_traj_path}")

    gt_path = os.path.join(traj_gt_dir, seq_traj_file)
    if not os.path.exists(gt_path):
        csv_gt_file = sequence + ".csv"
        gt_path = os.path.join(traj_gt_dir, csv_gt_file)
        if not os.path.exists(gt_path):
            continue

    eva_seq_file = sequence + ".zip"
    eva_seq_path = os.path.join(eva_seq_root, eva_seq_file)
    os.system(f"evo_ape tum {gt_path} {seq_traj_path} -a --save_results {eva_seq_path}")

# Uncomment the following lines if you want to generate a summary table
# table_file = os.path.join(eva_sum_root, "sum.csv")
# os.system(f"evo_res {eva_seq_root}/*.zip -p --save_table {table_file}")
