import platform
import struct
import sys
import os
import ctypes as ct

libb0 = None
prefix, suffix = 'lib', '.so'
if platform.system() in ('cli', 'Windows'):
    prefix, suffix = '', '.dll'
if platform.system() in ('Darwin', ):
    suffix = '.dylib'
for path in ('.', 'build', '../../build'):
    fullpath = os.path.join(os.path.dirname(__file__), path)
    if not os.path.isdir(fullpath): continue
    libb0_fullpath = os.path.join(fullpath, '%sb0%s' % (prefix, suffix))
    if os.path.exists(libb0_fullpath):
        libb0 = ct.CDLL(libb0_fullpath)
        break
if libb0 is None:
    raise RuntimeError('%sb0%s not found' % (prefix, suffix))

def _(n, ret, *args):
    # perform encoding/decoding for char* argument (use str to enable conversion)
    def _enc(v, t): return v.encode('ascii') if t == str else v
    def _dec(v, t): return v.decode('ascii') if t == str else v
    def _wrap(t): return ct.c_char_p if t == str else t
    # unwrapped CFUNCTYPE: (prefixed with _)
    globals()['_' + n] = ct.CFUNCTYPE(_wrap(ret), *[_wrap(arg) for arg in args])((n, libb0))
    # wrapped CFUNCTYPE: (performs string encoding/decoding)
    globals()[n] = lambda *args2: _dec(globals()['_' + n](*[_enc(arg, t) for t, arg in zip(args, args2)]), ret)

_("b0_init", ct.c_void_p, ct.POINTER(ct.c_int), ct.POINTER(ct.c_char_p))
_("b0_buffer_new", ct.c_void_p, ct.c_size_t)
_("b0_buffer_delete", None, ct.c_void_p)
_("b0_node_new", ct.c_void_p, str)
_("b0_node_delete", None, ct.c_void_p)
_("b0_node_init", None, ct.c_void_p)
_("b0_node_shutdown", None, ct.c_void_p)
_("b0_node_shutdown_requested", ct.c_int, ct.c_void_p)
_("b0_node_spin_once", None, ct.c_void_p)
_("b0_node_spin", None, ct.c_void_p)
_("b0_node_cleanup", None, ct.c_void_p)
_("b0_node_get_name", str, ct.c_void_p)
_("b0_node_get_state", ct.c_int, ct.c_void_p)
_("b0_node_get_context", ct.c_void_p, ct.c_void_p)
_("b0_node_hardware_time_usec", ct.c_longlong, ct.c_void_p)
_("b0_node_time_usec", ct.c_longlong, ct.c_void_p)
_("b0_node_log", None, ct.c_void_p, ct.c_int, str)
_("b0_publisher_new_ex", ct.c_void_p, ct.c_void_p, str, ct.c_int, ct.c_int)
_("b0_publisher_new", ct.c_void_p, ct.c_void_p, str)
_("b0_publisher_delete", None, ct.c_void_p)
_("b0_publisher_init", None, ct.c_void_p)
_("b0_publisher_cleanup", None, ct.c_void_p)
_("b0_publisher_spin_once", None, ct.c_void_p)
_("b0_publisher_get_topic_name", str, ct.c_void_p)
_("b0_publisher_publish", None, ct.c_void_p, ct.c_void_p, ct.c_size_t)
_("b0_publisher_log", None, ct.c_void_p, ct.c_int, str)
_("b0_subscriber_new_ex", ct.c_void_p, ct.c_void_p, str, ct.c_void_p, ct.c_int, ct.c_int)
_("b0_subscriber_new", ct.c_void_p, ct.c_void_p, str, ct.c_void_p)
_("b0_subscriber_delete", None, ct.c_void_p)
_("b0_subscriber_init", None, ct.c_void_p)
_("b0_subscriber_cleanup", None, ct.c_void_p)
_("b0_subscriber_spin_once", None, ct.c_void_p)
_("b0_subscriber_get_topic_name", str, ct.c_void_p)
_("b0_subscriber_log", None, ct.c_void_p, ct.c_int, str)
_("b0_subscriber_poll", ct.c_int, ct.c_void_p, ct.c_long)
_("b0_subscriber_read", ct.c_void_p, ct.c_void_p, ct.POINTER(ct.c_size_t))
_("b0_subscriber_set_option", ct.c_int, ct.c_void_p, ct.c_int, ct.c_int)
_("b0_service_client_new_ex", ct.c_void_p, ct.c_void_p, str, ct.c_int, ct.c_int)
_("b0_service_client_new", ct.c_void_p, ct.c_void_p, str)
_("b0_service_client_delete", None, ct.c_void_p)
_("b0_service_client_init", None, ct.c_void_p)
_("b0_service_client_cleanup", None, ct.c_void_p)
_("b0_service_client_spin_once", None, ct.c_void_p)
_("b0_service_client_get_service_name", str, ct.c_void_p)
_("b0_service_client_call", ct.c_void_p, ct.c_void_p, ct.c_void_p, ct.c_size_t, ct.POINTER(ct.c_size_t))
_("b0_service_client_set_option", ct.c_int, ct.c_void_p, ct.c_int, ct.c_int)
_("b0_service_client_log", None, ct.c_void_p, ct.c_int, str)
_("b0_service_server_new_ex", ct.c_void_p, ct.c_void_p, str, ct.c_void_p, ct.c_int, ct.c_int)
_("b0_service_server_new", ct.c_void_p, ct.c_void_p, str, ct.c_void_p)
_("b0_service_server_delete", None, ct.c_void_p)
_("b0_service_server_init", None, ct.c_void_p)
_("b0_service_server_cleanup", None, ct.c_void_p)
_("b0_service_server_spin_once", None, ct.c_void_p)
_("b0_service_server_get_service_name", str, ct.c_void_p)
_("b0_service_server_log", None, ct.c_void_p, ct.c_int, str)

def init():
    argc = ct.c_int(1)
    argc_p = ct.byref(argc)
    argv = ct.c_char_p(b'b0python')
    argv_p =ct.byref(argv)
    b0_init(argc_p, argv_p)
    
class Node:
    def __init__(self, name='node'):
        self._node = b0_node_new(name)

    def __del__(self):
        b0_node_delete(self._node)

    def init(self):
        b0_node_init(self._node)

    def shutdown(self):
        b0_node_shutdown(self._node)

    def shutdown_requested(self):
        return b0_node_shutdown_requested(self._node)

    def spin_once(self):
        b0_node_spin_once(self._node)

    def spin(self):
        b0_node_spin(self._node)

    def cleanup(self):
        b0_node_cleanup(self._node)

    def get_name(self):
        return b0_node_get_name(self._node)

    def get_state(self):
        return b0_node_get_state(self._node)

    def get_context(self):
        return b0_node_get_context(self._node)

    def hardware_time_usec(self):
        return b0_node_hardware_time_usec(self._node)

    def time_usec(self):
        return b0_node_time_usec(self._node)

    def log(self, level, message):
        b0_node_log(self._node, level, message)

class Publisher:
    def __init__(self, node, topic_name, managed=1, notify_graph=1):
        self._pub = b0_publisher_new_ex(node._node, topic_name, managed, notify_graph)

    def __del__(self):
        b0_publisher_delete(self._pub)

    def init(self):
        b0_publisher_init(self._pub)

    def cleanup(self):
        b0_publisher_cleanup(self._pub)

    def spin_once(self):
        b0_publisher_spin_once(self._pub)

    def get_topic_name(self):
        return b0_publisher_get_topic_name(self._pub)

    def publish(self, data):
        buf = ct.c_char_p(data)
        b0_publisher_publish(self._pub, buf, len(data))

    def log(self, level, message):
        b0_publisher_log(self._pub, level, message)

class Subscriber:
    def __init__(self, node, topic_name, callback, managed=1, notify_graph=1):
        def w(data, size):
            data_bytes = bytearray(ct.cast(data, ct.POINTER(ct.c_ubyte * size)).contents)
            return callback(data_bytes)
        self._cb = ct.CFUNCTYPE(None, ct.c_void_p, ct.c_size_t)(w)
        self._sub = b0_subscriber_new_ex(node._node, topic_name, self._cb, managed, notify_graph)

    def __del__(self):
        b0_subscriber_delete(self._sub)

    def init(self):
        b0_subscriber_init(self._sub)

    def cleanup(self):
        b0_subscriber_cleanup(self._sub)

    def spin_once(self):
        b0_subscriber_spin_once(self._sub)

    def get_topic_name(self):
        return b0_subscriber_get_topic_name(self._sub)

    def log(self, level, message):
        b0_subscriber_log(self._sub, level, message)
        
#    def set_conflate(self, conflate):
#        b0_subscriber_set_conflate(self._sub,conflate)

    def poll(self,timeout):
        return b0_subscriber_poll(self._sub,timeout)
        
    def read(self):
        outsz = ct.c_size_t()
        outbuf = b0_subscriber_read(self._sub, ct.byref(outsz))
        outarr = ct.cast(outbuf, ct.POINTER(ct.c_ubyte * outsz.value))
        rep_bytes = bytearray(outarr.contents)
        return rep_bytes
        
    def set_option(self,option,optionVal):
        rep = b0_subscriber_set_option(self._sub,option,optionVal)
        return rep
        
class ServiceClient:
    def __init__(self, node, topic_name, managed=1, notify_graph=1):
        self._cli = b0_service_client_new_ex(node._node, topic_name, managed, notify_graph)

    def __del__(self):
        b0_service_client_delete(self._cli)

    def init(self):
        b0_service_client_init(self._cli)

    def cleanup(self):
        b0_service_client_cleanup(self._cli)

    def spin_once(self):
        b0_service_client_spin_once(self._cli)

    def get_service_name(self):
        return b0_service_client_get_service_name(self._cli)

    def call(self, data):
        buf = ct.c_char_p(data)
        sz = len(data)
        outsz = ct.c_size_t()
        outbuf = b0_service_client_call(self._cli, buf, sz, ct.byref(outsz))
        outarr = ct.cast(outbuf, ct.POINTER(ct.c_ubyte * outsz.value))
        # print(bytearray(outarr.contents))
        rep_bytes = bytearray(outarr.contents)
        return rep_bytes
        
    def set_option(self,option,optionVal):
        rep = b0_service_client_set_option(self._cli,option,optionVal)
        return rep

    def log(self, level, message):
        b0_service_client_log(self._cli, level, message)

class ServiceServer:
    def __init__(self, node, topic_name, callback, managed=1, notify_graph=1):
        def w(data, size, outsize):
            req_bytes = bytearray(ct.cast(data, ct.POINTER(ct.c_ubyte * size)).contents)
            resp_bytes = callback(req_bytes)
            outsize[0] = len(resp_bytes)
            outdata = b0_buffer_new(outsize[0])
            outarr = ct.cast(outdata, ct.POINTER(ct.c_ubyte * outsize[0]))
            #for i, c in enumerate(resp_str): outarr.contents[i] = ord(c)
            ct.memmove(outarr, ct.c_char_p(resp_bytes), len(resp_bytes))
            return outdata
        self._cb = ct.CFUNCTYPE(ct.c_void_p, ct.c_void_p, ct.c_size_t, ct.POINTER(ct.c_size_t))(w)
        self._srv = b0_service_server_new_ex(node._node, topic_name, self._cb, managed, notify_graph)

    def __del__(self):
        b0_service_server_delete(self._srv)

    def init(self):
        b0_service_server_init(self._srv)

    def cleanup(self):
        b0_service_server_cleanup(self._srv)

    def spin_once(self):
        b0_service_server_spin_once(self._srv)

    def get_service_name(self):
        return b0_service_server_get_service_name(self._srv)

    def log(self, level, message):
        b0_service_server_log(self._srv, level, message)

