#ifndef A2LMODEL_H
#define A2LMODEL_H

#include <QString>
#include <QVector>
#include <QtGlobal>

// Minimal ASAP2/A2L reader: extracts the CHARACTERISTIC list (calibratable
// objects) from an A2L file so the GUI's calibration table can be built from
// the firmware description instead of a hardcoded list. Only the record
// layouts this project uses are understood (float32 / uint32 / uint8).

enum class A2lType { Float32, Uint32, Uint16, Uint8 };

inline int a2lTypeSize(A2lType t)
{
    switch (t) {
    case A2lType::Float32: return 4;
    case A2lType::Uint32:  return 4;
    case A2lType::Uint16:  return 2;
    case A2lType::Uint8:   return 1;
    }
    return 4;
}

struct A2lChar {
    QString name;          // CHARACTERISTIC identifier (e.g. GPIO_P00_00_enable)
    QString desc;          // quoted description that follows the identifier
    QString unit;          // PHYS_UNIT, empty if none
    quint32 addr = 0;      // ECU address from the VALUE line
    A2lType type = A2lType::Float32;
    double  lo   = 0.0;    // lower limit
    double  hi   = 0.0;    // upper limit
};

// One MEASUREMENT: a read-only signal in the firmware's measurement block.
// Unlike A2lChar these carry no record layout - the base type sits directly in
// the MEASUREMENT header - and they are grouped for display by name prefix.
struct A2lMeas {
    QString name;          // MEASUREMENT identifier (e.g. AhrsRoll)
    QString desc;          // quoted description that follows the identifier
    QString unit;          // PHYS_UNIT, empty if none
    quint32 addr = 0;      // ECU_ADDRESS
    A2lType type = A2lType::Float32;
    double  lo   = 0.0;
    double  hi   = 0.0;
    bool    isBitMask = false;  // BIT_MASK view (a single diagnostics bit)
};

class A2lModel {
public:
    // Parse every CHARACTERISTIC in the file. Returns the list (empty on
    // failure); *err, if given, holds a human-readable reason on failure.
    static QVector<A2lChar> parseFile(const QString &path, QString *err = nullptr);

    // Parse every MEASUREMENT in the file. Lets the Sensors tab be built from
    // the firmware description instead of a hardcoded signal list: adding a
    // MEASUREMENT to the A2L is enough to make it appear in the GUI.
    static QVector<A2lMeas> parseMeasurements(const QString &path, QString *err = nullptr);
};

#endif // A2LMODEL_H
