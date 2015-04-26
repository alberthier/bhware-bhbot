import logger

def check_type(obj, types):
    if not isinstance(obj, types):
        raise AssertionError("Expected {} type, got {} for object {}".format(types, type(obj), obj))
