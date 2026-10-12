import os
import yaml

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None


def load_config(filename='hw_config.yaml'):
    """
    Carica un file di configurazione YAML (es. 'hw_config.yaml' o 'sim_config.yaml').
    Cerca in ordine:
    1. Percorso assoluto se filename esiste
    2. Share directory di utils_pkg (install space)
    3. Source directory di utils_pkg (src/utils_pkg/config)
    4. Percorsi standard nel workspace
    """
    if os.path.isabs(filename) and os.path.exists(filename):
        with open(filename, 'r') as f:
            return yaml.safe_load(f)

    search_dirs = []

    # 1. Share directory di utils_pkg
    if get_package_share_directory is not None:
        try:
            share_dir = get_package_share_directory('utils_pkg')
            search_dirs.append(os.path.join(share_dir, 'config'))
        except Exception:
            pass


    # 2. Percorso relativo al modulo utils_pkg (src/utils_pkg/utils_pkg/ -> src/utils_pkg/config/)
    this_dir = os.path.dirname(os.path.abspath(__file__))
    search_dirs.append(os.path.join(os.path.dirname(this_dir), 'config'))

    # 3. Percorsi standard nel workspace
    search_dirs.extend([
        '/root/my_ros2_ws/src/utils_pkg/config',
        '/home/usainbot/Desktop/CocchiaAlessandro/guarDrone/my_ros2_ws/src/utils_pkg/config',
        os.path.expanduser('~/my_ros2_ws/src/utils_pkg/config'),
        os.path.expanduser('~/guarDrone/my_ros2_ws/src/utils_pkg/config'),
    ])

    for d in search_dirs:
        candidate = os.path.join(d, filename)
        if os.path.exists(candidate):
            with open(candidate, 'r') as f:
                return yaml.safe_load(f)

    raise FileNotFoundError(
        f"Impossibile trovare il file di configurazione '{filename}'.\n"
        f"Cartelle cercate: {search_dirs}"
    )
