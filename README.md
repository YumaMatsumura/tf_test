# tf_test

This package is designed for testing functionalities related to `tf` in ROS2.

## Installation

```bash
git clone https://github.com/YumaMatsumura/tf_test.git
cd tf_test
docker build -t ros2_tf_test .
```

## Usage

```bash
docker run -it --rm ros2_tf_test bash
```

**Regular Node + No Namespace**
```bash
ros2 launch tf_test tf_listener_test.launch.py use_composition:=False use_namespace:=False
```

**Regular Node + With Namespace**
```bash
ros2 launch tf_test tf_listener_test.launch.py use_composition:=False use_namespace:=True
```

**Composable Node + No Namespace**
```bash
ros2 launch tf_test tf_listener_test.launch.py use_composition:=True use_namespace:=False
```

**Composable Node + With Namespace**
```bash
ros2 launch tf_test tf_listener_test.launch.py use_composition:=True use_namespace:=True
```

## License

This package is licensed under the Apache-2.0 License.

