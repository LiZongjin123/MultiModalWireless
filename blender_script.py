import math
import os
import bpy
import yaml

class BlenderScript:
    def __init__(self, scene_dir, fbx_models_dir, blender_output_dir):
        self.__scene_dir = scene_dir
        self.__all_scenes_data = {}
        self.__fbx_models_dir = fbx_models_dir
        self.__is_insert_key_frames = True
        self.__blender_output_dir = blender_output_dir
        self.__keep_materials = {"itu_metal", "itu_very_dry_ground", "itu_marble",
                                 "itu_concrete", "itu_glass", "itu_medium_dry_ground"}
    def init(self):
        self.__load_scene()
        os.makedirs(self.__blender_output_dir, exist_ok=True)

    def __load_scene(self):
        scene_files = os.listdir(self.__scene_dir)
        scene_files.sort()

        for scene_file_name in scene_files:
            scene_file_path = os.path.join(self.__scene_dir, scene_file_name)
            with open(scene_file_path, 'r', encoding='utf-8') as f:
                scene_data = yaml.safe_load(f)
            scene_data_frame_id = scene_data["frame"]
            self.__all_scenes_data[scene_data_frame_id] = scene_data["actors"]

    def __clean_previous_actors(self):
        for obj in bpy.data.objects:
            if obj.name.startswith("Actor_"):
                bpy.data.objects.remove(obj, do_unlink=True)

    def __import_fbx_model(self, fbx_model_path):
        bpy.ops.import_scene.fbx(filepath=fbx_model_path)
        fbx_model = bpy.context.selected_objects
        return fbx_model[0]

    def __pose_transform(self, location, rotation):
        transformed_location = {}
        transformed_rotation = {}

        transformed_location["x"] = location["x"]
        transformed_location["y"] = -location["y"]
        transformed_location["z"] = location["z"]

        transformed_rotation["roll"] = -math.radians(rotation["roll"])
        transformed_rotation["pitch"] = -math.radians(rotation["pitch"])
        transformed_rotation["yaw"] = -math.radians(rotation["yaw"])

        return transformed_location, transformed_rotation


    def __replace_actor_materials(self, actor):
        material_name = "itu_metal"
        if material_name in bpy.data.materials:
            itu_material = bpy.data.materials[material_name]
        else:
            itu_material = bpy.data.materials.new(name=material_name)
            itu_material.use_nodes = True

        def apply_material(target):
            if target.type == 'MESH':
                target.data.materials.clear()
                target.data.materials.append(itu_material)

        apply_material(actor)
        for child in actor.children_recursive:
            apply_material(child)

    def __remove_material_not_in_keep_materials(self):
        materials_to_remove = [material for material in bpy.data.materials if material.name not in self.__keep_materials]

        for obj in bpy.data.objects:
            if obj.type == 'MESH':
                for i, slot in enumerate(obj.material_slots):
                    if slot.material and slot.material.name not in self.__keep_materials:
                        obj.material_slots[i].material = None

        for material in materials_to_remove:
            bpy.data.materials.remove(material)

    def __export_mitsuba_scene(self, frame_id):
        output_file_path = os.path.join(self.__blender_output_dir, f"{frame_id}.xml")
        bpy.ops.export_scene.mitsuba(
            filepath=output_file_path,
            export_ids=True,
            axis_up='Z',
            axis_forward='Y',
        )
        print(f"frame_id: {frame_id}")

    def main_script(self):
        frame_ids = sorted(self.__all_scenes_data.keys())
        current_blender_frame = 1
        for frame_id in frame_ids:
            actors_data = self.__all_scenes_data[frame_id]

            bpy.context.scene.frame_set(current_blender_frame)

            self.__clean_previous_actors()

            for actor_id, actor_data in actors_data.items():
                actor_location = actor_data["location"]
                actor_rotation = actor_data["rotation"]
                actor_bp_id = actor_data["bp_id"]

                fbx_model_path = os.path.join(self.__fbx_models_dir, f"{actor_bp_id}.FBX")
                fbx_model = self.__import_fbx_model(fbx_model_path)

                fbx_model.name = f"Actor_{actor_id}"
                fbx_model_location, fbx_model_rotation = self.__pose_transform(actor_location, actor_rotation)
                fbx_model.location = (fbx_model_location["x"],
                                      fbx_model_location["y"],
                                      fbx_model_location["z"])
                fbx_model.rotation_euler = (fbx_model_rotation["roll"],
                                            fbx_model_rotation["pitch"],
                                            fbx_model_rotation["yaw"])

                self.__replace_actor_materials(fbx_model)

                if self.__is_insert_key_frames:
                    fbx_model.keyframe_insert("location")
                    fbx_model.keyframe_insert("rotation_euler")

            self.__remove_material_not_in_keep_materials()

            self.__export_mitsuba_scene(frame_id)
            current_blender_frame += 1

scene_dir = r"D:\output\carla_output\scene"
fbx_models_dir = r"D:\actors"
blender_output_dir = r"D:\output\blender_output"

blender_script = BlenderScript(scene_dir,
                               fbx_models_dir,
                               blender_output_dir)
blender_script.init()
blender_script.main_script()