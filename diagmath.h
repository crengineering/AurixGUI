#ifndef DIAGMATH_H
#define DIAGMATH_H

#include <QString>
#include <QVector>
#include "a2lmodel.h"

// Pure logic behind the Diagnostics tab's word grouping. No QWidget, no XCP
// dependency here on purpose -- so this file (and only this file) is what
// diag_selfcheck links, the same isolation positionmath.h gives the position
// panel's math and attitudeglwidget.h/.cpp gives the handedness math.
//
// The tab used to hardcode a single word (diagStatus, Xcp_Data 0x70030024,
// see the retired DIAG_ADDR check in xcppanel.cpp). Firmware >= 1.19.30 adds
// a second one, navDiag, at the tail of Xcp_Fusion, with the exact same
// shape (N individual BIT_MASK views over one ULONG) -- the same
// generalisation the Sensors tab, the plot signal picker and the DFLASH tab
// already got: read whichever words the loaded A2L actually describes,
// never a hardcoded one (project non-negotiable #1).

// One row: a single diagnostic bit and its A2L description (or the built-in
// fallback text when no A2L is loaded at all).
struct DiagRow { int bit = 0; QString text; };

// One diagnostic word: every BIT_MASK MEASUREMENT that shares an
// ECU_ADDRESS, plus enough of one of them (wordMeas) to decode the whole
// word via A2lModel::decodeFrom -- a BIT_MASK measurement's own
// {addr, type} already cover the FULL word, the mask only matters when
// picking a single bit back out of the decoded value, so no separate
// "plain word" MEASUREMENT needs to exist for a group to be decodable (one
// happens to exist for diagStatus -- see DiagStatus in the A2L -- but
// buildDiagWordGroups() does not require it).
struct DiagWordGroup {
    quint32 addr = 0;
    QString label;              // "DiagStatus", "NavDiag", ... see below
    A2lMeas wordMeas;            // for A2lModel::decodeFrom
    QVector<DiagRow> rows;       // sorted by bit
};

// Groups every BIT_MASK MEASUREMENT in meas by its ECU_ADDRESS, one
// DiagWordGroup per distinct address (address order), rows in bit order.
//
// Word label: a plain (non-BIT_MASK) MEASUREMENT sharing the same address
// names the word precisely (e.g. "DiagStatus") and wins when present; else
// the shared prefix of the bit names up to (not including) their first '_'
// ("Diag_DTS_Undertemp" -> "Diag", "NavDiag_GnssUntrusted" -> "NavDiag") --
// still A2L-derived, never a hardcoded guess.
//
// fallbackRows/fallbackLabel are used ONLY when meas contains no BIT_MASK
// measurement whatsoever (no A2L loaded, or one that predates BIT_MASK
// views) -- the single legacy diagStatus bit list. Once even one A2L word
// exists every group comes from the A2L; the fallback is never mixed in
// alongside real groups.
QVector<DiagWordGroup> buildDiagWordGroups(const QVector<A2lMeas> &meas,
                                            const QVector<DiagRow> &fallbackRows,
                                            const QString &fallbackLabel);

// Exercises buildDiagWordGroups() -- the no-A2L fallback, a single-word A2L,
// and a two-word A2L fixture parsed through the REAL A2lModel::parseMeasurements
// path (a temp file, not hand-built structs) mirroring a future navDiag word
// that (per the dispatch) may not have its own plain-word MEASUREMENT --
// headless, no QApplication. Run by the diag_selfcheck CMake target.
bool diagSelfCheck(QString *report = nullptr);

#endif // DIAGMATH_H
