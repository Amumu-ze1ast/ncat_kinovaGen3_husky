# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from kortex_driver/SequenceTaskHandle.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import kortex_driver.msg

class SequenceTaskHandle(genpy.Message):
  _md5sum = "d86cef527dffeac4930486133afaaef1"
  _type = "kortex_driver/SequenceTaskHandle"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """
SequenceHandle sequence_handle
uint32 task_index
================================================================================
MSG: kortex_driver/SequenceHandle

uint32 identifier
uint32 permission"""
  __slots__ = ['sequence_handle','task_index']
  _slot_types = ['kortex_driver/SequenceHandle','uint32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       sequence_handle,task_index

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(SequenceTaskHandle, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.sequence_handle is None:
        self.sequence_handle = kortex_driver.msg.SequenceHandle()
      if self.task_index is None:
        self.task_index = 0
    else:
      self.sequence_handle = kortex_driver.msg.SequenceHandle()
      self.task_index = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.sequence_handle.identifier, _x.sequence_handle.permission, _x.task_index))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.sequence_handle is None:
        self.sequence_handle = kortex_driver.msg.SequenceHandle()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.sequence_handle.identifier, _x.sequence_handle.permission, _x.task_index,) = _get_struct_3I().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.sequence_handle.identifier, _x.sequence_handle.permission, _x.task_index))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.sequence_handle is None:
        self.sequence_handle = kortex_driver.msg.SequenceHandle()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.sequence_handle.identifier, _x.sequence_handle.permission, _x.task_index,) = _get_struct_3I().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I