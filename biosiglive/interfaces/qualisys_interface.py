"""
This file contains a wrapper for the python qualisys SDK.
"""
from .param import *
from typing import Union
from .generic_interface import GenericInterface
from ..enums import InverseKinematicsMethods, InterfaceType
import xml.etree.ElementTree as ET

try:
    import asyncio
    import qtm_rt
except ModuleNotFoundError:
    pass


class QualisysClient(GenericInterface):
    """
    Class for interfacing with the Qualisys system.
    """

    def __init__(self, system_rate: int, ip: str = "192.168.254.1", port: int = 22224, init_now=True):
        """
        Initialize the QualisysClient class.

        Parameters
        ----------
        system_rate: int
            Streaming rate of the Qualisys software.
        ip: str
            IP address of the Qualisys software.
        port: int
            Port of the Qualisys software.
        init_now: bool
            Whether to initialize the client now.
            Usefull if you want to pickle the interface as the qualisys SDK is not pickable (swig).
        """
        super(QualisysClient, self).__init__(ip=ip, system_rate=system_rate, interface_type=InterfaceType.QualisysClient)
        self.address =ip
        self.port= port
        self.qualisys_client = None
        self.acquisition_rate = None
        self.system_rate = system_rate
        self.devices = []
        self.imu = []
        self.marker_sets = []
        self.is_frame = False
        self.is_initialized = False
        if init_now:
            asyncio.create_task(self._init_client())


    async def _init_client(self):
        """
        Initialize the qualisys client.
        """
        print(f"Connection to Qualisys DataStreamSDK at : {self.address} ...")
        self.qualisys_client.Connect = await qtm_rt.connect(self.address)
        if self.qualisys_client.Connect is None:
            print("Error","Failed to connect")
            return

        print("Connected to Qualisys")
        self.is_initialized = True

        xml_general = await self.qualisys_client.Connect.get_parameters(parameters=["general"])
        generalinfo = ET.fromstring(xml_general) 
        frame_rate = int(generalinfo.find('.//Frequency').text)
        if self.system_rate != frame_rate():
            raise ValueError(
                f"qualisys system rate ({self.qualisys_client.get_frame_rate()}) does not match the system rate "
                f"({self.system_rate})."
            )

    async def create(cls, ip="192.168.254.1", system_rate=100, port=22224):
        self = cls(ip, system_rate, init_now=False)
        await self._init_client()
        return self

    async def add_device(
        self,
        nb_channels: int,
        device_type: Union[DeviceType, str] = DeviceType.Emg,
        data_buffer_size: int = None,
        name: str = None,
        rate: float = 2000,
        device_range: tuple = None,
        processing_method: Union[RealTimeProcessingMethod, OfflineProcessingMethod] = None,
        **process_kwargs,
    ):
        """
        Add a device to the qualisys system.

        Parameters
        ----------
        nb_channels: int
            Number of channels of the device.
        device_type: Union[DeviceType, str]
            Type of the device.
        data_buffer_size: int
            Size of the buffer for the device.
        name: str
            Name of the device.
        rate: float
            Rate of the device.
        device_range: tuple
            Range of the device.
        processing_method : Union[RealTimeProcessingMethod, OfflineProcessingMethod]
            Method used to process the data.
        **process_kwargs
            Keyword arguments for the processing method.
        """
        device_tmp = self._add_device(
            nb_channels, device_type, name, rate, device_range, processing_method, **process_kwargs
        )
        device_tmp.interface = self.interface_type
        if self.qualisys_client:
            self.component.append('Analog')
            xml_analog = await self.qualisys_client.Connect.get_parameters(parameters=["analog"])
            device_tmp.infos = ET.fromstring(xml_analog) 
        else: 
            device_tmp.infos = None 

        device_tmp.data_windows = data_buffer_size
        self.devices.append(device_tmp)

    async def add_forceplate(
        self,
        data_buffer_size: int = None,
        name: str = None,
        rate: float = 2000,
        device_range: tuple = None,
        processing_method: Union[RealTimeProcessingMethod, OfflineProcessingMethod] = None,
        **process_kwargs,
    ):
        """
        Add a device to the qualisys system.

        Parameters
        ----------
        nb_channels: int
            Number of channels of the device.
        device_type: Union[DeviceType, str]
            Type of the device.
        data_buffer_size: int
            Size of the buffer for the device.
        name: str
            Name of the device.
        rate: float
            Rate of the device.
        device_range: tuple
            Range of the device.
        processing_method : Union[RealTimeProcessingMethod, OfflineProcessingMethod]
            Method used to process the data.
        **process_kwargs
            Keyword arguments for the processing method.
        """
        force_tmp = self._add_forceplate(
            name, rate, device_range, processing_method, **process_kwargs
        )
        force_tmp.interface = self.interface_type
        if self.qualisys_client:
            xml_force = await self.qualisys_client.Connect.get_parameters(parameters=["force"])
            force_tmp.infos = ET.fromstring(xml_force)
            self.component.append('force')
        else: 
            force_tmp.infos = None 

        force_tmp.data_windows = data_buffer_size
        self.force.append(force_tmp)
 


    async def add_marker_set(
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
        Add markers set to stream from the qualisys system.

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
        if self.qualisys_client:
            markers_tmp.subject_name = subject_name #a changer quand je saurais comment recuperer nom du sujet avec qualisys
            xlm_mks = await connection.get_parameters(parameters=['3d'])
            MksInfo = ET.fromstring(xlm_mks)
            markers_tmp.marker_names =[label.find('Name').text for label in MksInfo.findall(".//Label")]
            self.component.append('3d')
        else:
            markers_tmp.subject_name = subject_name
            markers_tmp.marker_names = marker_names
        markers_tmp.data_windows = data_buffer_size
        self.marker_sets.append(markers_tmp)
        

    @staticmethod
    async def get_force_plate_data(self, PF_name: Union[str, list] = "all", channel_idx: Union[int, list] = (), get_frame: bool = True):
        if qualisys_client:
            headerf, forcespacket = self.packet.get_force()
            PF_names=headerf #TODO change to be ok
            if len(self.force) == 0:
                raise ValueError("No PF has been added to the qualisys system.")
            if not self.is_initialized:
                raise RuntimeError("Qualisys client is not initialized.")
            if get_frame:
                self.packet.framenumber
            all_device_data = []
            if subject_name and isinstance(subject_name, list):
                subject_name = [subject_name]
            if PF_names and isinstance(PF_names, list):
                PF_names = [PF_names]
            all_force_data = []
        

            for PF in enumerate(forcespacket, 1):
                forces.new_data = np.zeros((3, len(PF_names), self.packet.framenumber))
                count = 0
                for i, PF_name in enumerate(forcespacket, 1):
                    forces_data_tmp = forcespacket[i][:] #TO CHECK
                    forces.new_data[:, count, :] = np.array(forces_data_tmp)[:, np.newaxis]
                all_forces_data.append(forces.new_data)
                forces.append_data(forces.new_data)
            if len(all_forces_data) == 1:
                return all_forces_data[0]
            return all_forces_data    
        raise NotImplementedError("Force plate streaming is not implemented yet.")

    async def get_device_data(
        self, device_name: Union[str, list] = "all", channel_idx: Union[int, list] = (), get_frame: bool = True
    ):
        """
        Get the device data from qualisys.

        Parameters
        ----------
        device_name: str or list
            Name of the device or list of devices names.
        channel_idx: Union[int, str]
            Index of the channel to return.
        get_frame: bool
            Whether to get a new frame from the qualisys system.

        Returns
        -------
        device_data: list
            All asked device data.
        """
        if len(self.devices) == 0:
            raise ValueError("No device has been added to the qualisys system.")
        if not self.is_initialized:
            raise RuntimeError("Qualisys client is not initialized.")
        if get_frame:
            self.packet.framenumber
        all_device_data = []
        if not isinstance(device_name, list):
            device_name = [device_name]
        if channel_idx and not isinstance(channel_idx, list):
            channel_idx = [channel_idx]
        device_data = []
        headerdevice, Devicevalue = self.packet.get_analog
        for d, device in enumerate(self.devices):
            if device_name[0] == "all" or device.name in device_name:
                device.new_data = np.zeros((device.nb_channels, device.sample))
                count = 0
                for output_name, channel_name, unit in device.infos:
                    data_tmp, _ = self.packet.get_analog(device.name, output_name, channel_name)
                    device.new_data[count, :] = data_tmp
                    device.channel_names.append(channel_name)
                    count += 1
                    if count == device.nb_channels:
                        break
                if channel_idx:
                    device_data = np.zeros((len(channel_idx), device.sample))
                    for idx in range(device.nb_channels):
                        if idx in channel_idx:
                            device_data[channel_idx.index(idx), :] = device.new_data[idx, :]
                device_data = device_data if channel_idx else device.new_data
                device.append_data(device.new_data)
                all_device_data.append(device_data)
        if len(all_device_data) == 1:
            return all_device_data[0]
        return all_device_data

    async def get_marker_set_data(
        self, subject_name: Union[str, list] = None, marker_names: Union[str, list] = None, get_frame: bool = True
    ):
        """
        Get the markers data from qualisys.

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
        await self.connection.stream_frames(components=self.component, on_packet=queue.put_nowait)
        if len(self.marker_sets) == 0:
            raise ValueError("No marker set has been added to the qualisys system.")
        if not self.is_initialized:
            raise RuntimeError("qualisys client is not initialized.")
        if get_frame:
            self.packet.framenumber
        if subject_name and isinstance(subject_name, list):
            subject_name = [subject_name]
        if marker_names and isinstance(marker_names, list):
            marker_names = [marker_names]
        #occluded = []
        all_markers_data = []
        #all_occluded_data = []
        if subject_name:
            marker_sets = [None] * len(subject_name)
            for s, marker_set in enumerate(self.marker_sets):
                if marker_set.subject_name in subject_name:
                    marker_sets[subject_name.index(marker_set.subject_name)] = marker_set
            if marker_sets == [None]:
                raise RuntimeError("No subject of this name.")
        else:
            marker_sets = self.marker_sets

        for markers in marker_sets:
            markers.new_data = np.zeros((3, len(markers.marker_names), markers.sample))
            count = 0
            header, allmarkers_data_tmp= self.get_3d_markers() #TO check
            get_3d_markers_no_label
            for m, marker_name in enumerate(markers.marker_names):
                markers_data_tmp=allmarkers_data_tmp[m][:]
                markers.new_data[:, count, :] = np.array(markers_data_tmp)[:, np.newaxis]
                #occluded.append(occluded_tmp)
            if marker_names:
                markers_data = np.zeros((3, len(marker_names), markers.sample))
                for n, name in enumerate(markers.marker_names):
                    if name in marker_names:
                        markers_data[:, marker_names.index(name), :] = markers.new_data[:, n, :]
                all_markers_data.append(markers_data)
            else:
                all_markers_data.append(markers.new_data)
            markers.append_data(markers.new_data)
            all_occluded_data.append(occluded)
        if len(all_markers_data) == 1:
            return all_markers_data[0], all_occluded_data[0]
        return all_markers_data, all_occluded_data

    async def init_client(self):
        """
        Initialize the Qualisys client if it is not already initialized.
        This function has to be called before get frame from interface.
        """
        if self.is_initialized:
            raise RuntimeError("Qualisys client is already initialized.")
        else:
            self._init_client()
            xlm_analog = await connection.get_parameters(parameters=['analog'])
            AnalogInfo = ET.fromstring(xlm_analog)
            for d, device in enumerate(self.devices):
                if not device.infos:
                    device.infos = self.qualisys_client.GetDeviceOutputDetails(device.name) #a changer
            
            xlm_3d = await connection.get_parameters(parameters=['3d'])
            MksInfo = ET.fromstring(xlm_3d)
            nom_mks = [label.find('Name').text for label in MksInfo.findall(".//Label")]
            for m, marker_set in enumerate(self.marker_sets):
                if not marker_set.markers_names:
                    marker_set.markers_names = [label.find('Name').text for label in MksInfo.findall(".//Label")]
                if not marker_set.subject_name:
                    marker_set.subject_name = [label.find('Name').text for label in MksInfo.findall(".//Label")]

    async def get_latency(self) -> float:
        """
        Get the latency between the qualisys system and the qualisys SDK.

        Returns
        -------
        latency: float
            Latency between the qualisys system and the qualisys SDK.
        """
        if not self.is_initialized:
            raise RuntimeError("qualisys client is not initialized.")
        self.qualisys_client = 0 #voir comment obtenir la latence
        return self.qualisys_client

    async def get_frame(self) -> bool:
        """
        Get a new frame from the qualisys system.

        Returns
        -------
        bool
            True if there is a frame, False otherwise.
        """
        """
        if not self.is_initialized:
            raise RuntimeError("qualisys client is not initialized.")
        self.is_frame = self.qualisys_client.GetFrame()
        while self.is_frame is not True:
            self.is_frame = self.qualisys_client.GetFrame()
        return self.is_frame
        """ #TODO

    async def get_frame_number(self) -> int:
        """
        Get the last frame number.

        Returns
        -------
        frame_number: int
            Last frame number.
        """
        if not self.is_initialized:
            raise RuntimeError("qualisys client is not initialized.")
        return self.packet.framenumber

    async def get_kinematics_from_markers(
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
