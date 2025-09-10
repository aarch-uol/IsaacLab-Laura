# generated from rosidl_generator_py/resource/_idl.py.em
# with input from robot_state_message:srv/ReturnRobotState.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'joint_pos'
# Member 'gripper_pos'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ReturnRobotState_Request(type):
    """Metaclass of message 'ReturnRobotState_Request'."""

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
            module = import_type_support('robot_state_message')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'robot_state_message.srv.ReturnRobotState_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__return_robot_state__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__return_robot_state__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__return_robot_state__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__return_robot_state__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__return_robot_state__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ReturnRobotState_Request(metaclass=Metaclass_ReturnRobotState_Request):
    """Message class 'ReturnRobotState_Request'."""

    __slots__ = [
        '_joint_pos',
        '_gripper_pos',
        '_sm_state',
        '_time',
    ]

    _fields_and_field_types = {
        'joint_pos': 'sequence<float>',
        'gripper_pos': 'sequence<float>',
        'sm_state': 'int32',
        'time': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.joint_pos = array.array('f', kwargs.get('joint_pos', []))
        self.gripper_pos = array.array('f', kwargs.get('gripper_pos', []))
        self.sm_state = kwargs.get('sm_state', int())
        self.time = kwargs.get('time', int())

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
        if self.joint_pos != other.joint_pos:
            return False
        if self.gripper_pos != other.gripper_pos:
            return False
        if self.sm_state != other.sm_state:
            return False
        if self.time != other.time:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def joint_pos(self):
        """Message field 'joint_pos'."""
        return self._joint_pos

    @joint_pos.setter
    def joint_pos(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'joint_pos' array.array() must have the type code of 'f'"
            self._joint_pos = value
            return
        if __debug__:
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'joint_pos' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._joint_pos = array.array('f', value)

    @builtins.property
    def gripper_pos(self):
        """Message field 'gripper_pos'."""
        return self._gripper_pos

    @gripper_pos.setter
    def gripper_pos(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'gripper_pos' array.array() must have the type code of 'f'"
            self._gripper_pos = value
            return
        if __debug__:
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'gripper_pos' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._gripper_pos = array.array('f', value)

    @builtins.property
    def sm_state(self):
        """Message field 'sm_state'."""
        return self._sm_state

    @sm_state.setter
    def sm_state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'sm_state' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'sm_state' field must be an integer in [-2147483648, 2147483647]"
        self._sm_state = value

    @builtins.property
    def time(self):
        """Message field 'time'."""
        return self._time

    @time.setter
    def time(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'time' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'time' field must be an integer in [-2147483648, 2147483647]"
        self._time = value


# Import statements for member types

# Member 'joint_pos'
# Member 'gripper_pos'
# already imported above
# import array

# already imported above
# import builtins

# already imported above
# import math

# already imported above
# import rosidl_parser.definition


class Metaclass_ReturnRobotState_Response(type):
    """Metaclass of message 'ReturnRobotState_Response'."""

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
            module = import_type_support('robot_state_message')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'robot_state_message.srv.ReturnRobotState_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__return_robot_state__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__return_robot_state__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__return_robot_state__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__return_robot_state__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__return_robot_state__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ReturnRobotState_Response(metaclass=Metaclass_ReturnRobotState_Response):
    """Message class 'ReturnRobotState_Response'."""

    __slots__ = [
        '_joint_pos',
        '_gripper_pos',
        '_sm_state',
        '_time',
    ]

    _fields_and_field_types = {
        'joint_pos': 'sequence<float>',
        'gripper_pos': 'sequence<float>',
        'sm_state': 'int32',
        'time': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.joint_pos = array.array('f', kwargs.get('joint_pos', []))
        self.gripper_pos = array.array('f', kwargs.get('gripper_pos', []))
        self.sm_state = kwargs.get('sm_state', int())
        self.time = kwargs.get('time', int())

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
        if self.joint_pos != other.joint_pos:
            return False
        if self.gripper_pos != other.gripper_pos:
            return False
        if self.sm_state != other.sm_state:
            return False
        if self.time != other.time:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def joint_pos(self):
        """Message field 'joint_pos'."""
        return self._joint_pos

    @joint_pos.setter
    def joint_pos(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'joint_pos' array.array() must have the type code of 'f'"
            self._joint_pos = value
            return
        if __debug__:
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'joint_pos' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._joint_pos = array.array('f', value)

    @builtins.property
    def gripper_pos(self):
        """Message field 'gripper_pos'."""
        return self._gripper_pos

    @gripper_pos.setter
    def gripper_pos(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'gripper_pos' array.array() must have the type code of 'f'"
            self._gripper_pos = value
            return
        if __debug__:
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'gripper_pos' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._gripper_pos = array.array('f', value)

    @builtins.property
    def sm_state(self):
        """Message field 'sm_state'."""
        return self._sm_state

    @sm_state.setter
    def sm_state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'sm_state' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'sm_state' field must be an integer in [-2147483648, 2147483647]"
        self._sm_state = value

    @builtins.property
    def time(self):
        """Message field 'time'."""
        return self._time

    @time.setter
    def time(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'time' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'time' field must be an integer in [-2147483648, 2147483647]"
        self._time = value


class Metaclass_ReturnRobotState(type):
    """Metaclass of service 'ReturnRobotState'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('robot_state_message')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'robot_state_message.srv.ReturnRobotState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__return_robot_state

            from robot_state_message.srv import _return_robot_state
            if _return_robot_state.Metaclass_ReturnRobotState_Request._TYPE_SUPPORT is None:
                _return_robot_state.Metaclass_ReturnRobotState_Request.__import_type_support__()
            if _return_robot_state.Metaclass_ReturnRobotState_Response._TYPE_SUPPORT is None:
                _return_robot_state.Metaclass_ReturnRobotState_Response.__import_type_support__()


class ReturnRobotState(metaclass=Metaclass_ReturnRobotState):
    from robot_state_message.srv._return_robot_state import ReturnRobotState_Request as Request
    from robot_state_message.srv._return_robot_state import ReturnRobotState_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
