"""
This file is part of biosiglive. It contains a wrapper for the Vicon SDK for Python.
"""

import numpy as np
from .param import *
from typing import Union
from .generic_interface import GenericInterface
from ..enums import InverseKinematicsMethods, InterfaceType
from ..processing.msk_functions import MskFunctions

try:
    from vicon_dssdk import ViconDataStream as VDS
except ModuleNotFoundError:
    pass

try:
    import biorbd

    is_biorbd = True
except ModuleNotFoundError:
    is_biorbd = False


class ViconClient(GenericInterface):
    """
    Class for interfacing with the Vicon system.
    """

    def __init__(self, system_rate: int, ip: str = "127.0.0.1", port: int = 801, init_now=True):
        """
        Initialize the ViconClient class.
        Parameters
        ----------
        system_rate: int
            Streaming rate of the nexus software.
        ip: str
            IP address of the nexus software.
        port: int
            Port of the nexus software.
        """
        super(ViconClient, self).__init__(ip=ip, system_rate=system_rate, interface_type=InterfaceType.ViconClient)
        self.address = f"{ip}:{port}"

        self.vicon_client = None
        self.acquisition_rate = None
        self.system_rate = system_rate

        # Add possibility to initialize the client after, as swig objects are not pickable (multiprocessing).
        if init_now:
            self._init_client()

        self.devices = []
        self.imu = []
        self.marker_sets = []
        self.is_frame = False
        self.is_initialized = False

    def _init_client(self):
        print(f"Connection to ViconDataStreamSDK at : {self.address} ...")
        self.vicon_client = VDS.Client()
        self.vicon_client.Connect(self.address)
        print("Connected to Vicon.")
        self.is_initialized = True

        # Enable several data types
        self.vicon_client.EnableSegmentData()
        self.vicon_client.EnableDeviceData()
        self.vicon_client.EnableMarkerData()
        self.vicon_client.EnableUnlabeledMarkerData()
        self.get_frame()
        if self.system_rate != self.vicon_client.GetFrameRate():
            raise ValueError(
                f"Vicon system rate ({self.vicon_client.GetFrameRate()}) does not match the system rate "
                f"({self.system_rate})."
            )

    def add_device(
        self,
        nb_channels: int,
        device_type: Union[DeviceType, str] = DeviceType.Emg,
        data_buffer_size: int = None,
        name: str = None,
        rate: float = 2000,
        device_range: tuple = None,
        process_method: Union[RealTimeProcessingMethod, OfflineProcessingMethod] = None,
        **process_kwargs
    ):
        """
        Add a device to the Vicon system.
        Parameters
        ----------
        nb_channels: int
            Number of channels of the device.
        data_buffer_size: int
            Size of the buffer for the device.
        name: str
            Name of the device.
        device_type: Union[DeviceType, str]
            Type of the device.
        rate: float
            Rate of the device.
        device_range: tuple
            Range of the device.
        process_method : Union[RealTimeProcessingMethod, OfflineProcessingMethod]
            Method used to process the data.
        **process_kwargs
            Keyword arguments for the processing method.
        """
        device_tmp = self._add_device(nb_channels, name, device_type, rate, device_range, process_method, **process_kwargs)
        device_tmp.interface = self.interface_type
        if self.vicon_client:
            device_tmp.info = self.vicon_client.GetDeviceOutputDetails(name)
        else:
            device_tmp.info = None
        device_tmp.data_windows = data_buffer_size
        self.devices.append(device_tmp)

    def add_marker_set(
        self,
        nb_markers: int,
        name: str = None,
        data_buffer_size: int = None,
        marker_names: Union[str, list] = None,
        rate: float = 100,
        unlabeled: bool = False,
        subject_name: str = None,
        kinematics_method: InverseKinematicsMethods = None,
        **kin_method_kwargs,
    ):
        """
        Add markers set to stream from the Vicon system.
        Parameters
        ----------
        nb_markers: int
            Number of markers.
        name: str
            Name of the markers set.
        data_buffer_size: int
            Size of the buffer for the markers set.
        marker_names: Union[list, str]
            List of markers names.
        rate: int
            Rate of the markers set.
        unlabeled: bool
            Whether the markers set is unlabeled.
        subject_name: str
            Name of the subject. If None, the subject will be the first one in Nexus.
        kinematics_method: InverseKinematicsMethods
            Method used to compute the kinematics.
        **kin_method_kwargs
            Keyword arguments for the kinematics method.
        """
        if len(self.marker_sets) != 0:
            raise ValueError("Only one marker set can be added for now.")

        markers_tmp = self._add_marker_set(
            nb_markers=nb_markers,
            name=name,
            marker_names=marker_names,
            rate=rate,
            unlabeled=unlabeled,
            kinematics_method=kinematics_method,
            **kin_method_kwargs,
        )
        if self.vicon_client:
            markers_tmp.subject_name = subject_name if subject_name else self.vicon_client.GetSubjectNames()[0]
            markers_tmp.markers_names = (
                self.vicon_client.GetMarkerNames(markers_tmp.subject_name) if not marker_names else marker_names
            )
        else:
            markers_tmp.subject_name = subject_name
            markers_tmp.markers_names = marker_names
        markers_tmp.data_windows = data_buffer_size
        self.marker_sets.append(markers_tmp)

    @staticmethod
    def get_force_plate_data():
        raise NotImplementedError("Force plate streaming is not implemented yet.")

    def get_device_data(
        self, device_name: Union[str, list] = "all", channel_idx: Union[int, list] = (), get_frame: bool = True
    ):
        """
        Get the device data from Vicon.
        Parameters
        ----------
        device_name: str or list
            Name of the device or list of devices names.
        channel_idx: Union[int, str]
            Index of the channel to return.
        get_frame: bool
            Whether to get a new frame from the Vicon system.

        Returns
        -------
        device_data: list
            All asked device data.
        """
        if len(self.devices) == 0:
            raise ValueError("No device has been added to the Vicon system.")
        if not self.is_initialized:
            raise RuntimeError("Vicon client is not initialized.")
        if get_frame:
            self.get_frame()
        all_device_data = []
        if not isinstance(device_name, list):
            device_name = [device_name]

        if not isinstance(channel_idx, list):
            channel_idx = [channel_idx]

        for d, device in enumerate(self.devices):
            if device_name[0] == "all" or device.name in device_name:
                device.new_data = np.zeros((device.nb_channels, device.sample))
                device_data = np.zeros((len(channel_idx), device.sample))
                count = 0
                for output_name, chanel_name, unit in device.infos:
                    data_tmp, _ = self.vicon_client.GetDeviceOutputValues(device.name, output_name, chanel_name)
                    device.new_data[count, :] = data_tmp
                    device.chanel_names.append(chanel_name)
                    count += 1
                for idx in range(device.nb_channels):
                    if idx in channel_idx:
                        device_data[channel_idx.index(idx), :] = device.new_data[idx, :]
                device_data = device_data if channel_idx else device.new_data
                device.append_data(device.new_data)
                all_device_data.append(device_data)
        if len(all_device_data) == 1:
            return all_device_data[0]
        return all_device_data

    def get_marker_set_data(
        self, subject_name: Union[str, list] = None, marker_names: Union[str, list] = (), get_frame: bool = True
    ):
        """
        Get the markers data from Vicon.
        Parameters
        ----------
        subject_name: Union[str, list]
            Name of the subject. If None, the subject will be the first one in Nexus.
        marker_names: Union[str, list]
            List of markers names.
        get_frame: bool
            Whether to get a new frame or not.

        Returns
        -------
        markers_data: list
            All asked markers data.
        """
        if len(self.marker_sets) == 0:
            raise ValueError("No marker set has been added to the Vicon system.")
        if not self.is_initialized:
            raise RuntimeError("Vicon client is not initialized.")
        if get_frame:
            self.get_frame()
        if not isinstance(subject_name, list):
            subject_name = [subject_name]
        if not isinstance(marker_names, list):
            marker_names = [marker_names]
        occluded = []
        all_markers_data = []
        all_occluded_data = []
        if subject_name:
            marker_sets = [None] * len(subject_name)
            for s, marker_set in enumerate(self.marker_sets):
                if marker_set.subject_name in subject_name:
                    marker_sets[subject_name.index(marker_set.subject_name)] = marker_set
        else:
            marker_sets = self.marker_sets

        for markers in marker_sets:
            markers_data = np.zeros((3, len(marker_names), markers.sample))
            markers.new_data = np.zeros((3, len(markers.marker_names), markers.sample))
            count = 0
            for m, marker_name in enumerate(markers.marker_names):
                markers_data_tmp, occluded_tmp = self.vicon_client.GetMarkerGlobalTranslation(
                    markers.subject_name, marker_name
                )
                markers.new_data[:, count, :] = np.array(markers_data_tmp)[:, np.newaxis]
                occluded.append(occluded_tmp)
            for n, name in enumerate(markers.marker_names):
                if name in marker_names:
                    markers_data[:, marker_names.index(name), :] = markers.new_data[:, n, :]
            markers.append_data(markers.new_data)
            all_markers_data.append(markers_data)
            all_occluded_data.append(occluded)
        if len(all_markers_data) == 1:
            return all_markers_data[0], all_occluded_data[0]
        return all_markers_data, all_occluded_data

    def init_client(self):
        if self.is_initialized:
            raise RuntimeError("Vicon client is already initialized.")
        else:
            self._init_client()
            for d, device in enumerate(self.devices):
                if not device.infos:
                    device.infos = self.vicon_client.GetDeviceOutputDetails(device.name)
            for m, marker_set in enumerate(self.marker_sets):
                if not marker_set.markers_names:
                    marker_set.markers_names = self.vicon_client.GetMarkerNames(marker_set.subject_name)
                if not marker_set.subject_name:
                    marker_set.subject_name = self.vicon_client.GetSubjectNames()[0]

    def get_latency(self):
        if not self.is_initialized:
            raise RuntimeError("Vicon client is not initialized.")
        return self.vicon_client.GetLatencyTotal()

    def get_frame(self, init=False):
        if not self.is_initialized:
            raise RuntimeError("Vicon client is not initialized.")
        self.is_frame = self.vicon_client.GetFrame()
        while self.is_frame is not True:
            self.is_frame = self.vicon_client.GetFrame()
        return self.is_frame

    def get_frame_number(self):
        if not self.is_initialized:
            raise RuntimeError("Vicon client is not initialized.")
        return self.vicon_client.GetFrameNumber()

    def get_kinematics_from_markers(
        self,
        marker_set_name: str,
        model_path: str = None,
        method: Union[InverseKinematicsMethods, str] = InverseKinematicsMethods.BiorbdLeastSquare,
        custom_func: callable = None,
        **kwargs,
    ):
        """
        Get the kinematics from markers.
        Parameters
        ----------
        marker_set_name: str
            name of the markerset.
        model_path: str
            biorbd model of the kinematics.
        method: str
            Method to use to get the kinematics. Can be "kalman" or "custom".
        custom_func: function
            Custom function to get the kinematics.
        Returns
        -------
        kinematics: list
            List of kinematics.
        """
        marker_set_idx = [i for i, m in enumerate(self.marker_sets) if m.name == marker_set_name][0]
        return self.marker_sets[marker_set_idx].get_kinematics(model_path, method, custom_func=custom_func, **kwargs)
