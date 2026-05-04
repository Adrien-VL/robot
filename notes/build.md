When using the `symlink-install` flag you won't need to rebuild whenever you modify files such as URDF files (unless you add files).

Generally when you apply a change to a python file, with this tag you also don't need to rebuild, however you do if you change anything to setup.py or other config files (good rule of thumb).

```bash
colcon build --symlink-install
```