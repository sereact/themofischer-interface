ERROR_CODES = {
    -32000: {
        "code": "METHOD_NOT_FOUND",
        "description": "Requested method is not available",
        "severity": "",
        "resolution": ""
    },
    -32001: {
        "code": "DOOR_OPEN",
        "description": "The door of the picking cell is open.",
        "severity": "",
        "resolution": ""
    },
    -32002: {
        "code": "ROBOT_NOT_OPERATIONAL",
        "description": "The robot is not responding or is disconnected from the control system.",
        "severity": "",
        "resolution": ""
    },
    -32003: {
        "code": "EMERGENCY_STATE",
        "description": "The system is in an emergency state.",
        "severity": "",
        "resolution": ""
    },
    -32004: {
        "code": "PICK_FAILED",
        "description": "The pick operation failed.",
        "severity": "",
        "resolution": ""
    },
    -32005: {
        "code": "ITEM_DROPPED",
        "description": "The item was dropped during picking.",
        "severity": "",
        "resolution": ""
    },
    -32006: {
        "code": "LOAD_CARRIER_DROPPED",
        "description": "The load carrier was dropped during picking.",
        "severity": "",
        "resolution": ""
    },
    -32007: {
        "code": "BIN_EMPTY",
        "description": "The pick bin was empty.",
        "severity": "",
        "resolution": ""
    },
    -32008: {
        "code": "IN_EXECUTION",
        "description": "The robot is currently executing the previous request.",
        "severity": "",
        "resolution": ""
    },
    -32010: {
        "code": "ARGUMENT_ERROR",
        "description": "The arguments do not match the expected parameters.",
        "severity": "",
        "resolution": ""
    },
    -32011: {
        "code": "LOADCARRIERTYPE",
        "description": "The PICK request does not have LoadCarrierType.",
        "severity": "",
        "resolution": ""
    },
    -32012: {
        "code": "BARCODE_IS_NOT_READABLE",
        "description": "Barcode is not readable. The robot tried to read the barcode, but the barcode is not readable or no barcode is available.",
        "severity": "",
        "resolution": ""
    },
    -32013: {
        "code": "LOAD_CARRIER_TYPE_NOT_SUPPORTED",
        "description": "The load carrier type is not supported.",
        "severity": "",
        "resolution": ""
    },
    -32014: {
        "code": "PLACE_LOCATION_NOT_SUPPORTED",
        "description": "The place location is not supported.",
        "severity": "",
        "resolution": ""
    },
    -32015: {
        "code": "CANNOT_SCAN_BARCODE_OF_THE_TOTE",
        "description": "The barcode of the tote cannot be scanned.",
        "severity": "",
        "resolution": ""
    
    },
    -32016: {
        "code": "REQUEST_TIMEOUT",
        "description": "The request timed out.",
        "severity": "",
        "resolution": ""
    },
    -32017: {
        "code": "ITEM_IS_NOT_PICKABLE",
        "description": "The item is not pickable.",
        "severity": "",
        "resolution": ""
    },
    -32018: {
        "code": "PLACE_BIN_IS_FULL",
        "description": "Place bin is full.",
        "severity": "",
        "resolution": ""
    },
    -32019: {
        "code": "PICKCOUNT",
        "description": "PickCount is not integer.",
        "severity": "",
        "resolution": ""
    },
    -32020: {
        "code": "PICKING_SOFTWARE_IS_NOT_RUNNING",
        "description": "The picking software is not running. Please start the picking software.",
        "severity": "",
        "resolution": ""
    },
    -32021: {
        "code": "CAMERA_IS_NOT_RUNNING",
        "description": "The camera is not running. Please check the cameras.",
        "severity": "",
        "resolution": ""
    },
    -42001: {
        "code": "IO",
        "description": "IO device does not seem to be running.",
        "severity": "",
        "resolution": ""
    },
    -42002: {
        "code": "IK",
        "description": "Inverse Kinematics outside joint limits.",
        "severity": "",
        "resolution": ""
    },
    -42003: {
        "code": "PLANNING",
        "description": "Scalar product of the resulting TCP pose doesn't work.",
        "severity": "",
        "resolution": ""
    },
    -42004: {
        "code": "PLANNING",
        "description": "TCP Pose outside of operation area.",
        "severity": "",
        "resolution": ""
    },
    -42005: {
        "code": "PLANNING",
        "description": "Body outside operation area.",
        "severity": "",
        "resolution": ""
    },
    -42006: {
        "code": "PLANNING",
        "description": "Joint outside sensible limits.",
        "severity": "",
        "resolution": ""
    },
    -42007: {
        "code": "CALIBRATION",
        "description": "Robot is not ready for calibration.",
        "severity": "",
        "resolution": ""
    },
    -42008: {
        "code": "CANCEL",
        "description": "Motion was cancelled.",
        "severity": "",
        "resolution": ""
    },
    -42009: {
        "code": "RESTART",
        "description": "Hard restart needed.",
        "severity": "",
        "resolution": ""
    },
    -42010: {
        "code": "ROBOT_NOT_OPERATIONAL",
        "description": "Robot is not in automatic mode.",
        "severity": "",
        "resolution": ""
    },
    -42011: {
        "code": "SOFTWARE_ERROR",
        "description": "Software error.",
        "severity": "",
        "resolution": ""
    },
    -42012: {
        "code": "GRIPPER_FAILURE",
        "description": "Gripper failure.",
        "severity": "",
        "resolution": ""
    },
    -42013: {
        "code": "PICKING_PROCESS_FAIL",
        "description": "Picking process failed.",
        "severity": "",
        "resolution": ""
    },
    -42014: {
        "code": "CAMERA_FAILURE",
        "description": "Camera failure.",
        "severity": "",
        "resolution": ""
    },
    -42015: {
        "code": "TOTE_PROCESS_FAIL",
        "description": "Tote process failed.",
        "severity": "",
        "resolution": ""
    }
}
ERROR_CODE_MAP = {value["code"]: key for key, value in ERROR_CODES.items()}


ERROR_CODES_GERMAN = {
    -32000: {
        "code": "METHODE_NICHT_GEFUNDEN",
        "description": "Die angeforderte Methode ist nicht verfügbar.",
        "severity": "",
        "resolution": ""
    },
    -32001: {
        "code": "TÜR_OFFEN",
        "description": "Die Tür der Pickzelle ist offen.",
        "severity": "",
        "resolution": ""
    },
    -32002: {
        "code": "ROBOTER_NICHT_BETRIEBSBEREIT",
        "description": "Der Roboter ist nicht betriebsbereit.",
        "severity": "",
        "resolution": ""
    },
    -32003: {
        "code": "NOTFALLZUSTAND",
        "description": "Das System befindet sich im Notfallzustand.",
        "severity": "",
        "resolution": ""
    },
    -32004: {
        "code": "PICK_FEHLGESCHLAGEN",
        "description": "Der Pick-Vorgang ist fehlgeschlagen.",
        "severity": "",
        "resolution": ""
    },
    -32005: {
        "code": "OBJEKT_FALLEN_GELASSEN",
        "description": "Das Objekt wurde während des Picks fallen gelassen.",
        "severity": "",
        "resolution": ""
    },
    -32006: {
        "code": "LASTTRÄGER_FALLEN_GELASSEN",
        "description": "Der Lastträger wurde während des Picks fallen gelassen.",
        "severity": "",
        "resolution": ""
    },
    -32007: {
        "code": "BEHÄLTER_LEER",
        "description": "Der Pick-Behälter war leer.",
        "severity": "",
        "resolution": ""
    },
    -32008: {
        "code": "IN_AUSFÜHRUNG",
        "description": "Der Roboter führt derzeit die vorherige Anfrage aus.",
        "severity": "",
        "resolution": ""
    },
    -32010: {
        "code": "ARGUMENTFEHLER",
        "description": "Die Argumente stimmen nicht mit den erwarteten Parametern überein.",
        "severity": "",
        "resolution": ""
    },
    -32011: {
        "code": "LASTTRÄGERTYP",
        "description": "Die PICK-Anfrage hat keinen Lastträgertyp.",
        "severity": "",
        "resolution": ""
    },
    -32012: {
        "code": "BARCODE_NICHT_LESBAR",
        "description": "Der Barcode ist nicht lesbar. Der Roboter hat versucht, den Barcode zu lesen, aber der Barcode ist nicht lesbar oder nicht vorhanden.",
        "severity": "",
        "resolution": ""
    },
    -32013: {
        "code": "LASTTRÄGERTYP_NICHT_UNTERSTÜTZT",
        "description": "Der Lastträgertyp wird nicht unterstützt.",
        "severity": "",
        "resolution": ""
    },
    -32014: {
        "code": "ABLAGEORT_NICHT_UNTERSTÜTZT",
        "description": "Der Ablageort wird nicht unterstützt.",
        "severity": "",
        "resolution": ""
    },
    -32015: {
        "code": "BARCODE_DES_BEHÄLTERS_NICHT_SCANNBAR",
        "description": "Der Barcode des Behälters kann nicht gescannt werden.",
        "severity": "",
        "resolution": ""
    
    },
    -32016: {
        "code": "ANFRAGEZEITÜBERSCHREITUNG",
        "description": "Die Anfrage hat das Zeitlimit überschritten.",
        "severity": "",
        "resolution": ""
    },
    -32017: {
        "code": "OBJEKT_NICHT_AUFNEHMBAR",
        "description": "Das Objekt kann nicht aufgenommen werden.",
        "severity": "",
        "resolution": ""
    },
    -32018: {
        "code": "ABLAGEBEHÄLTER_VOLL",
        "description": "Der Ablagebehälter ist voll.",
        "severity": "",
        "resolution": ""
    },
    -32019: {
        "code": "PICK-ANZAHL",
        "description": "Pick-Anzahl ist keine ganze Zahl.",
        "severity": "",
        "resolution": ""
    },
    -32020: {
        "code": "PICKING-SOFTWARE_NICHT_GESTARTET",
        "description": "Die Picking-Software ist nicht gestartet. Bitte starten Sie die Picking-Software.",
        "severity": "",
        "resolution": ""
    },
    -32021: {
        "code": "KAMERA_NICHT_GESTARTET",
        "description": "Die Kamera ist nicht gestartet. Bitte überprüfen Sie die Kameras.",
        "severity": "",
        "resolution": ""
    },
    -42001: {
        "code": "IO",
        "description": "Das IO-Gerät scheint nicht zu laufen.",
        "severity": "",
        "resolution": ""
    },
    -42002: {
        "code": "IK",
        "description": "Inverse Kinematik liegt außerhalb der Gelenkgrenzen.",
        "severity": "",
        "resolution": ""
    },
    -42003: {
        "code": "PLANUNG",
        "description": "Das Skalarprodukt der resultierenden TCP-Position funktioniert nicht.",
        "severity": "",
        "resolution": ""
    },
    -42004: {
        "code": "PLANUNG",
        "description": "Die TCP-Position liegt außerhalb des Arbeitsbereichs.",
        "severity": "",
        "resolution": ""
    },
    -42005: {
        "code": "PLANUNG",
        "description": "Der Körper liegt außerhalb des Arbeitsbereichs.",
        "severity": "",
        "resolution": ""
    },
    -42006: {
        "code": "PLANUNG",
        "description": "Das Gelenk liegt außerhalb sinnvoller Grenzen.",
        "severity": "",
        "resolution": ""
    },
    -42007: {
        "code": "KALIBRIERUNG",
        "description": "Der Roboter ist nicht bereit für die Kalibrierung.",
        "severity": "",
        "resolution": ""
    },
    -42008: {
        "code": "ABBRUCH",
        "description": "Die Bewegung wurde abgebrochen.",
        "severity": "",
        "resolution": ""
    },
    -42009: {
        "code": "NEUSTART",
        "description": "Ein harter Neustart ist erforderlich.",
        "severity": "",
        "resolution": ""
    },
    -42010: {
        "code": "ROBOTER_NICHT_BETRIEBSBEREIT",
        "description": "Der Roboter ist nicht im Automatikmodus.",
        "severity": "",
        "resolution": ""
    }
}
# ERROR_CODES = {
#     -32000: {"title": "Method not found",  "description": "Requested method is not available"},
#     -32001: {"title": "Door Open", "description": "The door of the picking cell is open."},
#     -32002: {"title": "Robot Not Operational", "description": "The robot is not operational."},
#     -32003: {"title": "Emergency State", "description": "The system is in an emergency state."},
#     -32004: {"title": "Pick Failed", "description": "The pick operation failed."},
#     -32005: {"title": "Item Dropped", "description": "The item was dropped during picking."},
#     -32006: {"title": "Load carrier dropped", "description": "The load carrier was dropped during picking."},
#     -32007: {"title": "Bin Empty", "description": "The pick bin was empty."},
#     -32008: {"title": "In Execution", "description": "The robot is currently executing the previous request."},
#     -32010: {"title": "Argument Error", "description": "The arguments do not match the expected parameters."},
#     -32011: {"title": "LoadCarrierType", "description": "The PICK request does not have LoadCarrierType."},
#     -32012: {"title": "Barcode is not readable", "description": "Barcode is not readable. The robot tried to read the barcode, but the barcode is not readable or no barcode is available."},
#     -32013: {"title": "Load carrier type not supported", "description": "The load carrier type is not supported."},
#     -32014: {"title": "Place location not supported", "description": "The place location is not supported."},
#     -32015: {"title": "Cannot scan barcode of the tote", "description": "The barcode of the tote cannot be scanned."},
#     -32016: {"title": "Request timeout", "description": "The request timed out."},
#     -32017: {"title": "Item is not pickable", "description": "The item is not pickable."},
#     -32018: {"title": "Place bin is full", "description": "The place bin is full."},
#     -32019: {"title": "PickCount", "description": "PickCount is not integer."},
#     -32020: {"title": "Picking software is not running", "description": "The picking software is not running. Please start the picking software."},
#     -32021: {"title": "Camera is not running", "description": "The camera is not running. Please check the cameras."},
#     -42001: {"title": "IO", "description": "IO device does not seem to be running"},
#     -42002: {"title": "IK", "description": "Inverse Kinematics outside joint limits."},
#     -42003: {"title": "Planning", "description": "Scalar product of the resulting TCP pose doesn't work"},
#     -42004: {"title": "Planning", "description": "TCP Pose outside of operation area"},
#     -42005: {"title": "Planning", "description": "Body outside operation area"},
#     -42006: {"title": "Planning", "description": "Joint outside sensible limits"},
#     -42007: {"title": "Calibration", "description": "Robot is not ready for calibration"},
#     -42008: {"title": "Cancel", "description": "Motion was cancelled"},
#     -42009: {"title": "Restart", "description": "Hard restart needed"},
#     -42010: {"title": "Robot Not Operational", "description": "Robot is not in automatic mode"}
# }

# ERROR_CODES_GERMAN = {
#     -32000: {"title": "Methode nicht gefunden",  "description": "Die angeforderte Methode ist nicht verfügbar."},
#     -32001: {"title": "Tür offen", "description": "Die Tür der Pickzelle ist offen."},
#     -32002: {"title": "Roboter nicht betriebsbereit", "description": "Der Roboter ist nicht betriebsbereit."},
#     -32003: {"title": "Notfallzustand", "description": "Das System befindet sich im Notfallzustand."},
#     -32004: {"title": "Pick fehlgeschlagen", "description": "Der Pick-Vorgang ist fehlgeschlagen."},
#     -32005: {"title": "Objekt fallen gelassen", "description": "Das Objekt wurde während des Picks fallen gelassen."},
#     -32006: {"title": "Lastträger fallen gelassen", "description": "Der Lastträger wurde während des Picks fallen gelassen."},
#     -32007: {"title": "Behälter leer", "description": "Der Pick-Behälter war leer."},
#     -32008: {"title": "In Ausführung", "description": "Der Roboter führt derzeit die vorherige Anfrage aus."},
#     -32010: {"title": "Argumentfehler", "description": "Die Argumente stimmen nicht mit den erwarteten Parametern überein."},
#     -32011: {"title": "Lastträgertyp", "description": "Die PICK-Anfrage hat keinen Lastträgertyp."},
#     -32012: {"title": "Barcode nicht lesbar", "description": "Der Barcode ist nicht lesbar. Der Roboter hat versucht, den Barcode zu lesen, aber der Barcode ist nicht lesbar oder nicht vorhanden."},
#     -32013: {"title": "Lastträgertyp nicht unterstützt", "description": "Der Lastträgertyp wird nicht unterstützt."},
#     -32014: {"title": "Ablageort nicht unterstützt", "description": "Der Ablageort wird nicht unterstützt."},
#     -32015: {"title": "Barcode des Behälters nicht scannbar", "description": "Der Barcode des Behälters kann nicht gescannt werden."},
#     -32016: {"title": "Anfragezeitüberschreitung", "description": "Die Anfrage hat das Zeitlimit überschritten."},
#     -32017: {"title": "Objekt nicht aufnehmbar", "description": "Das Objekt kann nicht aufgenommen werden."},
#     -32018: {"title": "Ablagebehälter voll", "description": "Der Ablagebehälter ist voll."},
#     -32019: {"title": "Pick-Anzahl", "description": "Pick-Anzahl ist keine ganze Zahl."},
#     -32020: {"title": "Picking-Software nicht gestartet", "description": "Die Picking-Software ist nicht gestartet. Bitte starten Sie die Picking-Software."},
#     -32021: {"title": "Kamera nicht gestartet", "description": "Die Kamera ist nicht gestartet. Bitte überprüfen Sie die Kameras."},
#     -42001: {"title": "IO", "description": "Das IO-Gerät scheint nicht zu laufen."},
#     -42002: {"title": "IK", "description": "Inverse Kinematik liegt außerhalb der Gelenkgrenzen."},
#     -42003: {"title": "Planung", "description": "Das Skalarprodukt der resultierenden TCP-Position funktioniert nicht."},
#     -42004: {"title": "Planung", "description": "Die TCP-Position liegt außerhalb des Arbeitsbereichs."},
#     -42005: {"title": "Planung", "description": "Der Körper liegt außerhalb des Arbeitsbereichs."},
#     -42006: {"title": "Planung", "description": "Das Gelenk liegt außerhalb sinnvoller Grenzen."},
#     -42007: {"title": "Kalibrierung", "description": "Der Roboter ist nicht bereit für die Kalibrierung."},
#     -42008: {"title": "Abbruch", "description": "Die Bewegung wurde abgebrochen."},
#     -42009: {"title": "Neustart", "description": "Ein harter Neustart ist erforderlich."},
#     -42010: {"title": "Roboter nicht betriebsbereit", "description": "Der Roboter ist nicht im Automatikmodus."}
# }



NON_AUTOMATIC_RESETABLE_CODES = [-32001, -32002, -32003, -32006, -42009, -42010]

class JsonRpcError(Exception):
    def __init__(self, code, changed_bin_state=False):
        if code not in ERROR_CODES:
            raise ValueError(f"Unknown error code: {code}")
        
        self.code = code
        self.changed_bin_state = changed_bin_state
        self.error_code = ERROR_CODES[code]["code"]  # Renamed from "title" to "code"
        self.description = ERROR_CODES[code]["description"]
        self.severity = ERROR_CODES[code]["severity"]
        self.resolution = ERROR_CODES[code]["resolution"]

    def __dict__(self):
        return {
            "code": self.code,
            "error_code": self.error_code,  # Updated key to match ERROR_CODES
            "description": self.description,
            "severity": self.severity,
            "resolution": self.resolution
        }


# def json_rpc_error_check(func):
#     """
#     A decorator to wrap functions handling request/response,
#     and provide a unified error handling mechanism for JsonRpcError.
#     """
#     def wrapper(self, request, response, *args, **kwargs):
#         try:
#             # Call the original function (method) being decorated
#             return func(self, request, response, *args, **kwargs)
#         except JsonRpcError as e:
#             # Handle any JsonRpcError that occurs
#             response.raise_exception = True
#             response.error_code = e.code
#             response.exception_msg = e.description
#             response.exception_type = e.title
#             return response
#         except Exception as e:
#             response.raise_exception = True
#             response.exception_msg = str(e)
#             response.exception_type = "Internal Error"
#             return response
#     return wrapper
