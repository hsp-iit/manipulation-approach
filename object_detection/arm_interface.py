# SPDX-FileCopyrightText: 2026 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
# SPDX-License-Identifier: BSD-3-Clause
import threading

import yarp


class ArmInterface:
    """RPC client of the ergocub-cartesian-control arm controllers, one per arm.

    The controllers have no collision avoidance: callers must make sure that the straight line from
    the current hand position to the target is free.
    """

    def __init__(self, local_port_prefix, remote_rpc_ports, timeout):
        """remote_rpc_ports maps the arm name to the RPC port of its controller"""
        yarp.Network.init()
        self.remote_rpc_ports = remote_rpc_ports
        self._lock = threading.Lock()   # a port serves one RPC call at a time
        self.ports = {}
        for arm in remote_rpc_ports:
            port = yarp.Port()
            port.setTimeout(timeout)
            if not port.open(f"{local_port_prefix}/{arm}/rpc:o"):
                raise RuntimeError(f"Cannot open the local YARP port of {arm}")
            self.ports[arm] = port

    def connect(self):
        """Connects to the controllers that are not connected yet. Returns the arms not available"""
        missing = []
        for arm, remote in self.remote_rpc_ports.items():
            port = self.ports[arm]
            if port.getOutputCount() == 0 and not yarp.Network.connect(port.getName(), remote):
                missing.append(arm)
        return missing

    def _call(self, arm, name, *args):
        command = yarp.Bottle()
        command.addString(name)
        for value in args:
            command.addFloat64(value)
        reply = yarp.Bottle()
        with self._lock:
            written = self.ports[arm].write(command, reply)
        if not written or reply.size() == 0:
            raise RuntimeError(f"No reply to {name} from the {arm} controller")
        # The controllers reply with the vocabs ok and fail: Value.asBool() is true for both
        return reply.get(0).toString() == "ok"

    def go_to_position(self, arm, position, duration):
        """Starts moving the hand to position [x, y, z], in the controller root frame, keeping its orientation"""
        return self._call(arm, "go_to_position", *position, duration)

    def is_motion_done(self, arm):
        return self._call(arm, "is_motion_done")

    def stop(self, arm):
        return self._call(arm, "stop")

    def close(self):
        for port in self.ports.values():
            port.close()
