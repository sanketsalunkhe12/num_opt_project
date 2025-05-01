# generated from rosidl_generator_py/resource/_idl.py.em
# with input from uav_msgs:msg/ConsensusTraj.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ConsensusTraj(type):
    """Metaclass of message 'ConsensusTraj'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('uav_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'uav_msgs.msg.ConsensusTraj')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__consensus_traj
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__consensus_traj
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__consensus_traj
            cls._TYPE_SUPPORT = module.type_support_msg__msg__consensus_traj
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__consensus_traj

            from geometry_msgs.msg import Point
            if Point.__class__._TYPE_SUPPORT is None:
                Point.__class__.__import_type_support__()

            from geometry_msgs.msg import Vector3
            if Vector3.__class__._TYPE_SUPPORT is None:
                Vector3.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

            from std_msgs.msg import String
            if String.__class__._TYPE_SUPPORT is None:
                String.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ConsensusTraj(metaclass=Metaclass_ConsensusTraj):
    """Message class 'ConsensusTraj'."""

    __slots__ = [
        '_header',
        '_robot_name',
        '_waypoints',
        '_bern_coeffs',
        '_velocity_min',
        '_velocity_max',
        '_acceleration_min',
        '_acceleration_max',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'robot_name': 'std_msgs/String',
        'waypoints': 'sequence<geometry_msgs/Point>',
        'bern_coeffs': 'sequence<geometry_msgs/Point>',
        'velocity_min': 'geometry_msgs/Vector3',
        'velocity_max': 'geometry_msgs/Vector3',
        'acceleration_min': 'geometry_msgs/Vector3',
        'acceleration_max': 'geometry_msgs/Vector3',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'String'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point')),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        from std_msgs.msg import String
        self.robot_name = kwargs.get('robot_name', String())
        self.waypoints = kwargs.get('waypoints', [])
        self.bern_coeffs = kwargs.get('bern_coeffs', [])
        from geometry_msgs.msg import Vector3
        self.velocity_min = kwargs.get('velocity_min', Vector3())
        from geometry_msgs.msg import Vector3
        self.velocity_max = kwargs.get('velocity_max', Vector3())
        from geometry_msgs.msg import Vector3
        self.acceleration_min = kwargs.get('acceleration_min', Vector3())
        from geometry_msgs.msg import Vector3
        self.acceleration_max = kwargs.get('acceleration_max', Vector3())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.robot_name != other.robot_name:
            return False
        if self.waypoints != other.waypoints:
            return False
        if self.bern_coeffs != other.bern_coeffs:
            return False
        if self.velocity_min != other.velocity_min:
            return False
        if self.velocity_max != other.velocity_max:
            return False
        if self.acceleration_min != other.acceleration_min:
            return False
        if self.acceleration_max != other.acceleration_max:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def robot_name(self):
        """Message field 'robot_name'."""
        return self._robot_name

    @robot_name.setter
    def robot_name(self, value):
        if __debug__:
            from std_msgs.msg import String
            assert \
                isinstance(value, String), \
                "The 'robot_name' field must be a sub message of type 'String'"
        self._robot_name = value

    @builtins.property
    def waypoints(self):
        """Message field 'waypoints'."""
        return self._waypoints

    @waypoints.setter
    def waypoints(self, value):
        if __debug__:
            from geometry_msgs.msg import Point
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, Point) for v in value) and
                 True), \
                "The 'waypoints' field must be a set or sequence and each value of type 'Point'"
        self._waypoints = value

    @builtins.property
    def bern_coeffs(self):
        """Message field 'bern_coeffs'."""
        return self._bern_coeffs

    @bern_coeffs.setter
    def bern_coeffs(self, value):
        if __debug__:
            from geometry_msgs.msg import Point
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, Point) for v in value) and
                 True), \
                "The 'bern_coeffs' field must be a set or sequence and each value of type 'Point'"
        self._bern_coeffs = value

    @builtins.property
    def velocity_min(self):
        """Message field 'velocity_min'."""
        return self._velocity_min

    @velocity_min.setter
    def velocity_min(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'velocity_min' field must be a sub message of type 'Vector3'"
        self._velocity_min = value

    @builtins.property
    def velocity_max(self):
        """Message field 'velocity_max'."""
        return self._velocity_max

    @velocity_max.setter
    def velocity_max(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'velocity_max' field must be a sub message of type 'Vector3'"
        self._velocity_max = value

    @builtins.property
    def acceleration_min(self):
        """Message field 'acceleration_min'."""
        return self._acceleration_min

    @acceleration_min.setter
    def acceleration_min(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'acceleration_min' field must be a sub message of type 'Vector3'"
        self._acceleration_min = value

    @builtins.property
    def acceleration_max(self):
        """Message field 'acceleration_max'."""
        return self._acceleration_max

    @acceleration_max.setter
    def acceleration_max(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'acceleration_max' field must be a sub message of type 'Vector3'"
        self._acceleration_max = value
