def detect_model():
    with open('/proc/device-tree/model') as f:
        model = f.read()
    return model