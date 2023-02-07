import os
# The starting number for the increment
folderName = "./measurementsMireHorizontale"

# The starting number for the increment
start = 1
deleted_indices = []

for i in range(1, 50):
    yaml_file = os.path.join(folderName, f"pose_cPo_{i}.yaml")
    if os.path.exists(yaml_file):
        # Rename the image, configuration, pose_fPe, and pose_cPo files to reflect the increment
        image_file = os.path.join(folderName, f"image-{i}.png")
        os.rename(image_file, os.path.join(folderName, f"image-{start}.png"))
        configuration_file = os.path.join(folderName, f"configuration_{i}")
        os.rename(configuration_file, os.path.join(folderName, f"configuration_{start}"))
        pose_fPe_file = os.path.join(folderName, f"pose_fPe_{i}.yaml")
        os.rename(pose_fPe_file, os.path.join(folderName, f"pose_fPe_{start}.yaml"))
        os.rename(yaml_file, os.path.join(folderName, f"pose_cPo_{start}.yaml"))
        start += 1
    else:
        # Delete the corresponding image, configuration, and pose_fPe files
        image_file = os.path.join(folderName, f"image-{i}.png")
        os.remove(image_file)
        deleted_indices.append(i)
        configuration_file = os.path.join(folderName, f"configuration_{i}")
        os.remove(configuration_file)
        deleted_indices.append(i)
        pose_fPe_file = os.path.join(folderName, f"pose_fPe_{i}.yaml")
        os.remove(pose_fPe_file)
        deleted_indices.append(i)

# Print the deleted indices
print("Deleted indices:")
for index in deleted_indices:
    print(index)
