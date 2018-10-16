/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtBluetooth module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:LGPL21$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see http://www.qt.io/terms-conditions. For further
** information use the contact form at http://www.qt.io/contact-us.
**
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 2.1 or version 3 as published by the Free
** Software Foundation and appearing in the file LICENSE.LGPLv21 and
** LICENSE.LGPLv3 included in the packaging of this file. Please review the
** following information to ensure the GNU Lesser General Public License
** requirements will be met: https://www.gnu.org/licenses/lgpl.html and
** http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html.
**
** As a special exception, The Qt Company gives you certain additional
** rights. These rights are described in The Qt Company LGPL Exception
** version 1.1, included in the file LGPL_EXCEPTION.txt in this package.
**
** $QT_END_LICENSE$
**
****************************************************************************/

#ifndef QBLUETOOTHUUID_H
#define QBLUETOOTHUUID_H

#include <QtBluetooth/qbluetoothglobal.h>

#include <QtCore/QtGlobal>
#include <QtCore/QMetaType>
#include <QtCore/QUuid>

#include <QtCore/QDebug>

QT_BEGIN_NAMESPACE

struct quint128
{
    quint8 data[16];
};

class Q_BLUETOOTH_EXPORT QBluetoothUuid : public QUuid
{
public:
    enum ProtocolUuid {
        Sdp = 0x0001,
        Udp = 0x0002,
        Rfcomm = 0x0003,
        Tcp = 0x0004,
        TcsBin = 0x0005,
        TcsAt = 0x0006,
        Att = 0x0007,
        Obex = 0x0008,
        Ip = 0x0009,
        Ftp = 0x000A,
        Http = 0x000C,
        Wsp = 0x000E,
        Bnep = 0x000F,
        Upnp = 0x0010,
        Hidp = 0x0011,
        HardcopyControlChannel = 0x0012,
        HardcopyDataChannel = 0x0014,
        HardcopyNotification = 0x0016,
        Avctp = 0x0017,
        Avdtp = 0x0019,
        Cmtp = 0x001B,
        UdiCPlain = 0x001D,
        McapControlChannel = 0x001E,
        McapDataChannel = 0x001F,
        L2cap = 0x0100
    };

    enum ServiceClassUuid {
        ServiceDiscoveryServer = 0x1000,
        BrowseGroupDescriptor = 0x1001,
        PublicBrowseGroup = 0x1002,
        SerialPort = 0x1101,
        LANAccessUsingPPP = 0x1102,
        DialupNetworking = 0x1103,
        IrMCSync = 0x1104,
        ObexObjectPush = 0x1105,
        OBEXFileTransfer = 0x1106,
        IrMCSyncCommand = 0x1107,
        Headset = 0x1108,
        AudioSource = 0x110a,
        AudioSink = 0x110b,
        AV_RemoteControlTarget = 0x110c,
        AdvancedAudioDistribution = 0x110d,
        AV_RemoteControl = 0x110e,
        AV_RemoteControlController = 0x110f,
        HeadsetAG = 0x1112,
        PANU = 0x1115,
        NAP = 0x1116,
        GN = 0x1117,
        DirectPrinting = 0x1118,
        ReferencePrinting = 0x1119,
        BasicImage = 0x111a,
        ImagingResponder = 0x111b,
        ImagingAutomaticArchive = 0x111c,
        ImagingReferenceObjects = 0x111d,
        Handsfree = 0x111e,
        HandsfreeAudioGateway = 0x111f,
        DirectPrintingReferenceObjectsService = 0x1120,
        ReflectedUI = 0x1121,
        BasicPrinting = 0x1122,
        PrintingStatus = 0x1123,
        HumanInterfaceDeviceService = 0x1124,
        HardcopyCableReplacement = 0x1125,
        HCRPrint = 0x1126,
        HCRScan = 0x1127,
        SIMAccess = 0x112d,
        PhonebookAccessPCE = 0x112e,
        PhonebookAccessPSE = 0x112f,
        PhonebookAccess = 0x1130,
        HeadsetHS = 0x1131,
        MessageAccessServer = 0x1132,
        MessageNotificationServer = 0x1133,
        MessageAccessProfile = 0x1134,
        GNSS = 0x1135,
        GNSSServer = 0x1136,
        Display3D = 0x1137,
        Glasses3D = 0x1138,
        Synchronization3D = 0x1139,
        MPSProfile = 0x113a,
        MPSService = 0x113b,
        PnPInformation = 0x1200,
        GenericNetworking = 0x1201,
        GenericFileTransfer = 0x1202,
        GenericAudio = 0x1203,
        GenericTelephony = 0x1204,
        VideoSource = 0x1303,
        VideoSink = 0x1304,
        VideoDistribution = 0x1305,
        HDP = 0x1400,
        HDPSource = 0x1401,
        HDPSink = 0x1402,
        GenericAccess = 0x1800,
        GenericAttribute = 0x1801,
        ImmediateAlert = 0x1802,
        LinkLoss = 0x1803,
        TxPower = 0x1804,
        CurrentTimeService = 0x1805,
        ReferenceTimeUpdateService = 0x1806,
        NextDSTChangeService = 0x1807,
        Glucose = 0x1808,
        HealthThermometer = 0x1809,
        DeviceInformation = 0x180a,
        HeartRate = 0x180d,
        PhoneAlertStatusService = 0x180e,
        BatteryService = 0x180f,
        BloodPressure = 0x1810,
        AlertNotificationService = 0x1811,
        HumanInterfaceDevice = 0x1812,
        ScanParameters = 0x1813,
        RunningSpeedAndCadence = 0x1814,
        CyclingSpeedAndCadence = 0x1816,
        CyclingPower = 0x1818,
        LocationAndNavigation = 0x1819,
        EnvironmentalSensing = 0x181a,
        BodyComposition = 0x181b,
        UserData = 0x181c,
        WeightScale = 0x181d,
        BondManagement = 0x181e,
        ContinuousGlucoseMonitoring = 0x181f
    };

    enum CharacteristicType {
        DeviceName = 0x2a00,
        Appearance = 0x2a01,
        PeripheralPrivacyFlag = 0x2a02,
        ReconnectionAddress = 0x2a03,
        PeripheralPreferredConnectionParameters = 0x2a04,
        ServiceChanged = 0x2a05,
        AlertLevel = 0x2a06,
        TxPowerLevel = 0x2a07,
        DateTime = 0x2a08,
        DayOfWeek = 0x2a09,
        DayDateTime = 0x2a0a,
        /* 0x2a0b not defined */
        ExactTime256 = 0x2a0c,
        DSTOffset = 0x2a0d,
        TimeZone = 0x2a0e,
        LocalTimeInformation = 0x2a0f,
        /* 0x2a10 not defined */
        TimeWithDST = 0x2a11,
        TimeAccuracy = 0x2a12,
        TimeSource = 0x2a13,
        ReferenceTimeInformation = 0x2a14,
        /* 0x2a15 not defined */
        TimeUpdateControlPoint = 0x2a16,
        TimeUpdateState = 0x2a17,
        GlucoseMeasurement = 0x2a18,
        BatteryLevel = 0x2a19,
        /* 0x2a1a not defined */
        /* 0x2a1b not defined */
        TemperatureMeasurement = 0x2a1c,
        TemperatureType = 0x2a1d,
        IntermediateTemperature = 0x2a1e,
        /* 0x2a1f not defined */
        /* 0x2a20 not defined */
        MeasurementInterval = 0x2a21,
        BootKeyboardInputReport = 0x2a22,
        SystemID = 0x2a23,
        ModelNumberString = 0x2a24,
        SerialNumberString = 0x2a25,
        FirmwareRevisionString = 0x2a26,
        HardwareRevisionString = 0x2a27,
        SoftwareRevisionString = 0x2a28,
        ManufacturerNameString = 0x2a29,
        IEEE1107320601RegulatoryCertificationDataList = 0x2a2a,
        CurrentTime = 0x2a2b,
        MagneticDeclination = 0x2a2c,
        /* 0x2a2d not defined */
        /* 0x2a2e not defined */
        /* 0x2a2f not defined */
        /* 0x2a30 not defined */
        ScanRefresh = 0x2a31,
        BootKeyboardOutputReport = 0x2a32,
        BootMouseInputReport = 0x2a33,
        GlucoseMeasurementContext = 0x2a34,
        BloodPressureMeasurement = 0x2a35,
        IntermediateCuffPressure = 0x2a36,
        HeartRateMeasurement = 0x2a37,
        BodySensorLocation = 0x2a38,
        HeartRateControlPoint = 0x2a39,
        /* 0x2a3a not defined */
        /* 0x2a3b not defined */
        /* 0x2a3c not defined */
        /* 0x2a3d not defined */
        /* 0x2a3e not defined */
        AlertStatus = 0x2a3f,
        RingerControlPoint = 0x2a40,
        RingerSetting = 0x2a41,
        AlertCategoryIDBitMask = 0x2a42,
        AlertCategoryID = 0x2a43,
        AlertNotificationControlPoint = 0x2a44,
        UnreadAlertStatus = 0x2a45,
        NewAlert = 0x2a46,
        SupportedNewAlertCategory = 0x2a47,
        SupportedUnreadAlertCategory = 0x2a48,
        BloodPressureFeature = 0x2a49,
        HIDInformation = 0x2a4a,
        ReportMap = 0x2a4b,
        HIDControlPoint = 0x2a4c,
        Report = 0x2a4d,
        ProtocolMode = 0x2a4e,
        ScanIntervalWindow = 0x2a4f,
        PnPID = 0x2a50,
        GlucoseFeature = 0x2a51,
        RecordAccessControlPoint = 0x2a52,
        RSCMeasurement = 0x2a53,
        RSCFeature = 0x2a54,
        SCControlPoint = 0x2a55,
        /* 0x2a56 not defined */
        /* 0x2a57 not defined */
        /* 0x2a58 not defined */
        /* 0x2a59 not defined */
        /* 0x2a5a not defined */
        CSCMeasurement = 0x2a5b,
        CSCFeature = 0x2a5c,
        SensorLocation = 0x2a5d,
        /* 0x2a5e not defined */
        /* 0x2a5f not defined */
        /* 0x2a60 not defined */
        /* 0x2a61 not defined */
        /* 0x2a62 not defined */
        CyclingPowerMeasurement = 0x2a63,
        CyclingPowerVector = 0x2a64,
        CyclingPowerFeature = 0x2a65,
        CyclingPowerControlPoint = 0x2a66,
        LocationAndSpeed = 0x2a67,
        Navigation = 0x2a68,
        PositionQuality = 0x2a69,
        LNFeature = 0x2a6a,
        LNControlPoint = 0x2a6b,
        Elevation = 0x2a6c,
        Pressure = 0x2a6d,
        Temperature = 0x2a6e,
        Humidity = 0x2a6f,
        TrueWindSpeed = 0x2a70,
        TrueWindDirection = 0x2a71,
        ApparentWindSpeed = 0x2a72,
        ApparentWindDirection = 0x2a73,
        GustFactor = 0x2a74,
        PollenConcentration = 0x2a75,
        UVIndex = 0x2a76,
        Irradiance = 0x2a77,
        Rainfall = 0x2a78,
        WindChill = 0x2a79,
        HeatIndex = 0x2a7a,
        DewPoint = 0x2a7b,
        /* 0x2a7c not defined */
        DescriptorValueChanged = 0x2a7d,
        AerobicHeartRateLowerLimit = 0x2a7e,
        AerobicThreshold = 0x2a7f,
        Age = 0x2a80,
        AnaerobicHeartRateLowerLimit = 0x2a81,
        AnaerobicHeartRateUpperLimit = 0x2a82,
        AnaerobicThreshold = 0x2a83,
        AerobicHeartRateUpperLimit = 0x2a84,
        DateOfBirth = 0x2a85,
        DateOfThresholdAssessment = 0x2a86,
        EmailAddress = 0x2a87,
        FatBurnHeartRateLowerLimit = 0x2a88,
        FatBurnHeartRateUpperLimit = 0x2a89,
        FirstName = 0x2a8a,
        FiveZoneHeartRateLimits = 0x2a8b,
        Gender = 0x2a8c,
        HeartRateMax = 0x2a8d,
        Height = 0x2a8e,
        HipCircumference = 0x2a8f,
        LastName = 0x2a90,
        MaximumRecommendedHeartRate = 0x2a91,
        RestingHeartRate = 0x2a92,
        SportTypeForAerobicAnaerobicThresholds = 0x2a93,
        ThreeZoneHeartRateLimits = 0x2a94,
        TwoZoneHeartRateLimits = 0x2a95,
        VO2Max = 0x2a96,
        WaistCircumference = 0x2a97,
        Weight = 0x2a98,
        DatabaseChangeIncrement = 0x2a99,
        UserIndex = 0x2a9a,
        BodyCompositionFeature = 0x2a9b,
        BodyCompositionMeasurement = 0x2a9c,
        WeightMeasurement = 0x2a9d,
        WeightScaleFeature = 0x2a9e,
        UserControlPoint = 0x2a9f,
        MagneticFluxDensity2D = 0x2aa0,
        MagneticFluxDensity3D = 0x2aa1,
        Language = 0x2aa2,
        BarometricPressureTrend = 0x2aa3
    };

    enum DescriptorType {
        UnknownDescriptorType = 0x0,
        CharacteristicExtendedProperties = 0x2900,
        CharacteristicUserDescription = 0x2901,
        ClientCharacteristicConfiguration = 0x2902,
        ServerCharacteristicConfiguration = 0x2903,
        CharacteristicPresentationFormat = 0x2904,
        CharacteristicAggregateFormat = 0x2905,
        ValidRange = 0x2906,
        ExternalReportReference = 0x2907,
        ReportReference = 0x2908,
        /* 0x2909 not defined */
        /* 0x290a not defined */
        EnvironmentalSensingConfiguration = 0x290b,
        EnvironmentalSensingMeasurement = 0x290c,
        EnvironmentalSensingTriggerSetting = 0x290d
    };

    QBluetoothUuid();
    QBluetoothUuid(ProtocolUuid uuid);
    QBluetoothUuid(ServiceClassUuid uuid);
    QBluetoothUuid(CharacteristicType uuid);
    QBluetoothUuid(DescriptorType uuid);
    explicit QBluetoothUuid(quint16 uuid);
    explicit QBluetoothUuid(quint32 uuid);
    explicit QBluetoothUuid(quint128 uuid);
    explicit QBluetoothUuid(const QString &uuid);
    QBluetoothUuid(const QBluetoothUuid &uuid);
    QBluetoothUuid(const QUuid &uuid);
    ~QBluetoothUuid();

    bool operator==(const QBluetoothUuid &other) const;

    int minimumSize() const;

    quint16 toUInt16(bool *ok = 0) const;
    quint32 toUInt32(bool *ok = 0) const;
    quint128 toUInt128() const;

    static QString serviceClassToString(ServiceClassUuid uuid);
    static QString protocolToString(ProtocolUuid uuid);
    static QString characteristicToString(CharacteristicType uuid);
    static QString descriptorToString(DescriptorType uuid);
};

#ifndef QT_NO_DEBUG_STREAM
/// TODO: Move implementation to .cpp, uninline and add Q_BLUETOOTH_EXPORT for Qt 6
inline QDebug operator<<(QDebug debug, const QBluetoothUuid &uuid)
{
    debug << uuid.toString();
    return debug;
}
#endif

QT_END_NAMESPACE

Q_DECLARE_METATYPE(QBluetoothUuid)

#endif
