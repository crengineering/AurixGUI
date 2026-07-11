#ifndef A2LMODEL_H
#define A2LMODEL_H

#include <QString>
#include <QVector>
#include <QtGlobal>

// Minimal ASAP2/A2L reader: extracts the CHARACTERISTIC list (calibratable
// objects) from an A2L file so the GUI's calibration table can be built from
// the firmware description instead of a hardcoded list. Only the record
// layouts this project uses are understood (float32 / uint32 / uint8).

enum class A2lType { Float32, Uint32, Uint8 };

inline int a2lTypeSize(A2lType t)
{
    switch (t) {
    case A2lType::Float32: return 4;
    case A2lType::Uint32:  return 4;
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

class A2lModel {
public:
    // Parse every CHARACTERISTIC in the file. Returns the list (empty on
    // failure); *err, if given, holds a human-readable reason on failure.
    static QVector<A2lChar> parseFile(const QString &path, QString *err = nullptr);
};

#endif // A2LMODEL_H
