"""Small controlled UE import boundary; does not simulate an engine acceptance run."""

from types import SimpleNamespace


def import_engine(returned_paths, directory_assets=()):
    class Mesh:
        def get_bounds(self):
            return SimpleNamespace(origin=SimpleNamespace(x=0, y=0, z=0),
                                   box_extent=SimpleNamespace(x=15, y=11.1, z=4.635))

        def get_path_name(self):
            return '/Game/Old/Existing.Existing'

    class Task:
        def __init__(self):
            self.values = {'imported_object_paths': list(returned_paths)}

        def set_editor_property(self, key, value):
            self.values[key] = value

        def get_editor_property(self, key):
            return self.values[key]

    mesh = Mesh()
    state = SimpleNamespace(unrelated_dirty=True, imports=0, loads=[])

    def load(path):
        state.loads.append(path)
        return mesh

    def save_directory(*args, **kwargs):
        state.unrelated_dirty = False
        return True

    def import_tasks(tasks):
        state.imports += len(tasks)

    library = SimpleNamespace(does_asset_exist=lambda path: False,
                              list_assets=lambda *args, **kwargs: list(directory_assets),
                              load_asset=load, save_directory=save_directory)
    registry = SimpleNamespace(get_assets_by_path=lambda *args, **kwargs:
                               [SimpleNamespace(get_asset=lambda: mesh)])
    api = SimpleNamespace(StaticMesh=Mesh, AssetImportTask=Task, EditorAssetLibrary=library,
                          AssetToolsHelpers=SimpleNamespace(get_asset_tools=lambda:
                                                           SimpleNamespace(import_asset_tasks=import_tasks)),
                          AssetRegistryHelpers=SimpleNamespace(get_asset_registry=lambda: registry),
                          load_asset=load, SystemLibrary=SimpleNamespace(get_engine_version=lambda: 'controlled-boundary'))
    return api, state
