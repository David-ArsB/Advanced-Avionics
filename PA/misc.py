def detect_model():
    '''
    Detect computer board. Returns board model.
    '''

    with open('/proc/device-tree/model') as f:
        model = f.read()
    return model
