import ctrlxdatalayer
from api_helper.ctrlx_data_layer_helper import get_provider, get_client
import os
# SPDX-FileCopyrightText: Bosch Rexroth AG
#
# SPDX-License-Identifier: MIT

import ctrlxdatalayer
from comm.datalayer import NodeClass
from ctrlxdatalayer.provider import Provider
from ctrlxdatalayer.provider_node import (
    ProviderNode,
    ProviderNodeCallbacks,
    NodeCallback,
)
from ctrlxdatalayer.variant import Result, Variant
from ctrlxdatalayer.metadata_utils import (
    MetadataBuilder,
    AllowedOperation,
    ReferenceType,
)
import flatbuffers
from vision.error_codes import ErrorCodes
from location_transform.location_data import Result as FB_Result 
from location_transform.location_data import LocationData

def read_write(address,type_address): 
    """create_metadata"""
    builder = MetadataBuilder(allowed=AllowedOperation.READ
                                  | AllowedOperation.WRITE)
    builder = builder.set_display_name(address)
    builder = builder.set_node_class(NodeClass.NodeClass.Variable)
    builder.add_reference(ReferenceType.read(), type_address)
    builder.add_reference(ReferenceType.write(), type_address)
    return builder.build()

def read_only(address, type_address):
    builder = MetadataBuilder(allowed=AllowedOperation.READ)
    builder = builder.set_display_name(address)
    builder = builder.set_node_class(NodeClass.NodeClass.Variable)
    builder.add_reference(ReferenceType.read(), type_address)
    return builder.build()
    
class Node:
    def __init__(self, provider: Provider, nodeAddress: str, typeAddress: str,
                 initialValue: Variant, meta_build):
        """__init__"""
        self._cbs = ProviderNodeCallbacks(
            self.__on_create,
            self.__on_remove,
            self.__on_browse,
            self.__on_read,
            self.__on_write,
            self.__on_metadata,
        )

        self._providerNode = ProviderNode(self._cbs)
        self._provider = provider
        self._nodeAddress = nodeAddress
        self._typeAddress = typeAddress
        self._data = initialValue
        self._metadata = meta_build(nodeAddress,typeAddress)


    def register_node(self):
        """register_node"""
        return self._provider.register_node(self._nodeAddress,
                                            self._providerNode)

    def unregister_node(self):
        """unregister_node"""
        self._provider.unregister_node(self._nodeAddress)
        self._metadata.close()
        self._data.close()

    def set_value(self, value: Variant):
        """set_value"""
        self._data = value

    def get_value(self): 
        return self._data 

    def __on_create(
        self,
        userdata: ctrlxdatalayer.clib.userData_c_void_p,
        address: str,
        data: Variant,
        cb: NodeCallback,
    ):
        """__on_create"""
        cb(Result.OK, data)

    def __on_remove(
        self,
        userdata: ctrlxdatalayer.clib.userData_c_void_p,
        address: str,
        cb: NodeCallback,
    ):
        """__on_remove"""
        cb(Result.UNSUPPORTED, None)

    def __on_browse(
        self,
        userdata: ctrlxdatalayer.clib.userData_c_void_p,
        address: str,
        cb: NodeCallback,
    ):
        """__on_browse"""
        with Variant() as new_data:
            new_data.set_array_string([])
            cb(Result.OK, new_data)

    def __on_read(
        self,
        userdata: ctrlxdatalayer.clib.userData_c_void_p,
        address: str,
        data: Variant,
        cb: NodeCallback,
    ):
        """__on_read"""
        new_data = self._data
        cb(Result.OK, new_data)

    def __on_write(
        self,
        userdata: ctrlxdatalayer.clib.userData_c_void_p,
        address: str,
        data: Variant,
        cb: NodeCallback,
    ):
        """__on_write"""
        if self._data.get_type() != data.get_type():
            cb(Result.TYPE_MISMATCH, None)
            return

        result, self._data = data.clone()
        cb(Result.OK, self._data)


    def __on_metadata(
        self,
        userdata: ctrlxdatalayer.clib.userData_c_void_p,
        address: str,
        cb: NodeCallback,
    ):
        """__on_metadata"""
        cb(Result.OK, self._metadata)

class CtrlxDlAPi():
    def __init__(self): 
        self.__provider = None
        self.__result_node = None
        self.__request_node = None
        self.__busy_node = None
        self.__client = None
        self.__done_node = None
        self.__error_node = None
        self.__error_code_node = None
        pass
    def start_sys(self, datalayer_system): 
        self.__provider, connection_string = get_provider(
        datalayer_system, ip="192.168.1.1", ssl_port=443)
        self.__client, connection_string = get_client( datalayer_system, ip="192.168.1.1", ssl_port=443)
        if self.__provider is not None and self.__client is not None: 
            self.__provider.start()
            return True 
        return False 
    
    def create_end_points(self): 
        root_node = 'Location_Transform'
        snap_path = os.getenv("SNAP")
        if snap_path is None:
            # Debug environment
            bfbs_path = os.path.join("./bfbs/", "location_data.bfbs")
        else:
            # snap environment
            bfbs_path = os.path.join(snap_path, "location_data.bfbs")

        # Register Flatbuffer type
        type_sampleSchema_inertialValue = "types/transform_loc/location_data"
        result = self.__provider.register_type(type_sampleSchema_inertialValue,
                                            bfbs_path)
        if result != Result.OK:
            return 
        builder = flatbuffers.Builder() 
        FB_Result.ResultStart(builder)
        builder.Finish(FB_Result.ResultEnd(builder))
        initial_value = Variant()
        initial_value.set_flatbuffers(builder.Bytes)
        self.__result_node = Node(self.__provider,root_node + '/Locations','types/transform_loc/location_data',
                           initial_value,read_only)
        self.__result_node.register_node()

        bool_value = Variant()
        bool_value.set_bool8(False)
        self.__request_node = Node(self.__provider, root_node + '/Control/Request', 'types/datalayer/bool8', bool_value,
                                   read_write)
        self.__request_node.register_node()

        bool_value = Variant() 
        bool_value.set_bool8(False)
        self.__busy_node = Node(self.__provider, root_node + '/Status/Busy', 'types/datalayer/bool8', bool_value,
                                   read_only)
        self.__busy_node.register_node()
        bool_value = Variant() 
        bool_value.set_bool8(False)
        self.__done_node = Node(self.__provider, root_node + '/Status/Done', 'types/datalayer/bool8', bool_value,
                                   read_only)
        self.__done_node.register_node()

        bool_value = Variant() 
        bool_value.set_bool8(False)
        self.__error_node = Node(self.__provider, root_node + '/Status/Error', 'types/datalayer/bool8', bool_value,
                                   read_only)
        self.__error_node.register_node()

        str_value = Variant() 
        str_value.set_string(ErrorCodes.NO_ERROR.name)
        self.__error_code_node = Node(self.__provider, root_node + '/Status/ErrorDescription', 'types/datalayer/string', str_value,
                                   read_only)
        self.__error_code_node.register_node()
        
    def get_request(self): 
        return self.__request_node.get_value().get_bool8()

    def write_busy(self, data): 
        bool_value = Variant() 
        bool_value.set_bool8(data)
        self.__busy_node.set_value(bool_value)
    
    def write_done(self,data): 
        bool_value = Variant() 
        bool_value.set_bool8(data)
        self.__done_node.set_value(bool_value)
    
    def write_error(self,data): 
        bool_value = Variant() 
        bool_value.set_bool8(data)
        self.__error_node.set_value(bool_value)

    def write_error_code(self,data:ErrorCodes): 
        str_value = Variant() 
        str_value.set_string(data.name)
        self.__error_code_node.set_value(str_value)

    def write_locations(self,locations): 
        builder = flatbuffers.Builder() 
        count = len(locations)

        FB_Result.StartLocationsVector(builder,count)

        # Prepend each struct in REVERSE order
        for location in reversed(locations):
            LocationData.CreateLocationData(
                builder,
                location['x'],
                location['y'],
                location['angle'],
                location['score'],
                location['class_index']
    )

        locations_vector = builder.EndVector()
        FB_Result.ResultStart(builder)
        FB_Result.AddLocations(builder,locations_vector)
        builder.Finish(FB_Result.ResultEnd(builder))
        var_result = Variant() 
        ok = var_result.set_flatbuffers(builder.Output())
        self.__result_node.set_value(var_result)

    def read_image(self,image_node): 
       ok, img =  self.__client.read_sync(image_node)
       return img.get_data() 
    
    def read_node(self, adr) -> Variant:
        ok, node = self.__client.read_sync(adr)
        return node
    
    def write_node(self, adr,data) -> Variant:
        ok, node = self.__client.write_sync(adr,data)
        return node
    
    def is_connected(self) -> bool: 
        return self.__provider.is_connected() and self.__client.is_connected() 
    
    def is_client_connected(self)-> bool: 
        return self.__client.is_connected()
    
    def is_provider_connected(self)-> bool: 
        return self.__provider.is_connected() 
    
    def __del__(self): 
        if self.__provider is not None:
            self.__provider.unregister_type('types/transform_loc/location_data')
            self.__provider.close()
            self.__provider.stop()
        if self.__client is not None: 
            self.__client.close()
    

 