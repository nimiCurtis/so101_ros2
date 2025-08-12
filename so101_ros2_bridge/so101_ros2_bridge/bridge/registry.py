ROBOT_FACTORY_REGISTRY = {}


def register_robot(robot_type: str):
    def decorator(factory_fn):
        ROBOT_FACTORY_REGISTRY[robot_type] = factory_fn
        return factory_fn

    return decorator
