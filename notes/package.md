**Creating a package:**

```bash
run ros2 pkg create --build-type ament_python --node-name node package_name
```

**Entrypoint:** In `setup.py` do:
```python
entry_points={
    'console_scripts': [
        'package_name = package_name.node:main'
    ],
},
```