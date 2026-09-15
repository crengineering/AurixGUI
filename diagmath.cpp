#include "diagmath.h"

#include <algorithm>
#include <QTemporaryFile>
#include <QTextStream>

namespace {

// Word label when no plain (non-BIT_MASK) measurement shares the address:
// the shared prefix of the bit names up to (not including) the first '_'.
QString prefixLabel(const QString &name)
{
    const int us = name.indexOf(QLatin1Char('_'));
    return us > 0 ? name.left(us) : name;
}

} // namespace

QVector<DiagWordGroup> buildDiagWordGroups(const QVector<A2lMeas> &meas,
                                            const QVector<DiagRow> &fallbackRows,
                                            const QString &fallbackLabel)
{
    QVector<DiagWordGroup> groups;

    for (const A2lMeas &mm : meas) {
        if (!mm.isBitMask || mm.bitMask == 0)
            continue;

        int gi = -1;
        for (int i = 0; i < groups.size(); ++i) {
            if (groups[i].addr == mm.addr) { gi = i; break; }
        }
        if (gi < 0) {
            DiagWordGroup g;
            g.addr     = mm.addr;
            g.wordMeas = mm;                 // addr+type are all decodeFrom needs
            g.label    = prefixLabel(mm.name);   // provisional, see below
            groups.append(g);
            gi = groups.size() - 1;
        }

        int bit = 0;
        while (bit < 31 && ((mm.bitMask >> bit) & 1u) == 0u)
            ++bit;
        groups[gi].rows.append({bit, mm.desc.isEmpty() ? mm.name : mm.desc});
    }

    // A plain (non-BIT_MASK) measurement at the same address names the word
    // precisely (e.g. DiagStatus) and overrides the bit-name-prefix guess.
    for (const A2lMeas &mm : meas) {
        if (mm.isBitMask)
            continue;
        for (DiagWordGroup &g : groups) {
            if (g.addr == mm.addr) { g.label = mm.name; break; }
        }
    }

    for (DiagWordGroup &g : groups) {
        std::sort(g.rows.begin(), g.rows.end(),
                  [](const DiagRow &a, const DiagRow &b) { return a.bit < b.bit; });
    }
    std::sort(groups.begin(), groups.end(),
              [](const DiagWordGroup &a, const DiagWordGroup &b) { return a.addr < b.addr; });

    if (groups.isEmpty()) {
        DiagWordGroup g;
        g.label = fallbackLabel;
        g.rows  = fallbackRows;
        groups.append(g);
    }

    return groups;
}

namespace {

bool checkCase(QString *report, bool ok, const QString &name)
{
    if (report)
        *report += (ok ? QStringLiteral("[PASS] ") : QStringLiteral("[FAIL] ")) + name + "\n";
    return ok;
}

} // namespace

bool diagSelfCheck(QString *report)
{
    bool allOk = true;

    const QVector<DiagRow> fallback = {{0, "fallback bit 0"}, {1, "fallback bit 1"}};

    // No A2L / no BIT_MASK measurements at all -> exactly the fallback group,
    // never mixed with anything real.
    {
        const QVector<DiagWordGroup> g = buildDiagWordGroups({}, fallback, "diagStatus");
        allOk &= checkCase(report, g.size() == 1 && g[0].label == "diagStatus"
                                        && g[0].rows.size() == 2,
                            "fallback: empty A2L -> exactly one fallback group");
    }

    // Single word, built by hand (no file I/O needed for this case): a plain
    // word measurement plus two bit views at the same address -- the plain
    // measurement's own name must win as the label.
    {
        QVector<A2lMeas> meas;
        A2lMeas word; word.name = "DiagStatus"; word.addr = 0x24; word.isBitMask = false;
        meas.append(word);
        A2lMeas b0; b0.name = "Diag_A"; b0.addr = 0x24; b0.isBitMask = true; b0.bitMask = 0x1; b0.desc = "bit A";
        A2lMeas b1; b1.name = "Diag_B"; b1.addr = 0x24; b1.isBitMask = true; b1.bitMask = 0x2; b1.desc = "bit B";
        meas.append(b0);
        meas.append(b1);

        const QVector<DiagWordGroup> g = buildDiagWordGroups(meas, fallback, "diagStatus");
        allOk &= checkCase(report, g.size() == 1 && g[0].label == "DiagStatus"
                                        && g[0].addr == 0x24
                                        && g[0].rows.size() == 2
                                        && g[0].rows[0].bit == 0 && g[0].rows[0].text == "bit A"
                                        && g[0].rows[1].bit == 1 && g[0].rows[1].text == "bit B",
                            "single word: plain-measurement name wins as the label, rows bit-sorted");
    }

    // Defensive corner case, hand-built: a word with BIT_MASK views but NO
    // plain word measurement of its own -- the label must fall back to the
    // bit names' shared prefix rather than going unlabelled or crashing. Not
    // today's real firmware (NavDiag below DOES have a plain measurement,
    // like DiagStatus), but a future diagnostics word might not, and this
    // path must still degrade sensibly.
    {
        QVector<A2lMeas> meas;
        A2lMeas b0; b0.name = "FooDiag_BarBroken"; b0.addr = 0x40; b0.isBitMask = true; b0.bitMask = 0x1; b0.desc = "bar broken";
        meas.append(b0);

        const QVector<DiagWordGroup> g = buildDiagWordGroups(meas, fallback, "diagStatus");
        allOk &= checkCase(report, g.size() == 1 && g[0].label == "FooDiag" && g[0].addr == 0x40
                                        && g[0].rows.size() == 1,
                            "no-plain-word corner case: label falls back to the bit names' shared prefix");
    }

    // Two words, parsed through the REAL A2L text path (A2lModel::parseMeasurements
    // against a temp file, not hand-built structs), verbatim from the
    // regenerated A2L on AurixTricore branch feat/nav-filter-strand @ af563f2
    // (fw 1.19.30, cross-checked against that file directly, read-only,
    // 2026-09-15): a second diagnostics word `NavDiag` at the Xcp_Fusion
    // tail (0x700305FC, DiagStatus's own 0x70030024 is now full at 32/32
    // bits), WITH its own plain word measurement, exactly mirroring
    // DiagStatus's shape -- so this exercises the plain-measurement-name
    // path, not the prefix-fallback one (that is the corner case above).
    {
        QTemporaryFile f;
        allOk &= checkCase(report, f.open(), "two-word fixture: temp file opens");
        if (f.isOpen()) {
            {
                QTextStream out(&f);
                out <<
                    "/begin MEASUREMENT DiagStatus \"diagnostics bitmask, see DIAGNOSTICS.md\"\n"
                    "  ULONG NO_COMPU_METHOD 0 0 0 4294967295\n"
                    "  ECU_ADDRESS 0x70030024\n"
                    "/end MEASUREMENT\n"
                    "\n"
                    "/begin MEASUREMENT Diag_DTS_Undertemp \"DTS temperature below dtsMin\"\n"
                    "  ULONG NO_COMPU_METHOD 0 0 0 1\n"
                    "  ECU_ADDRESS 0x70030024\n"
                    "  BIT_MASK 0x00000001\n"
                    "/end MEASUREMENT\n"
                    "\n"
                    "/begin MEASUREMENT Diag_DTS_Overtemp \"DTS temperature above dtsMax\"\n"
                    "  ULONG NO_COMPU_METHOD 0 0 0 1\n"
                    "  ECU_ADDRESS 0x70030024\n"
                    "  BIT_MASK 0x00000002\n"
                    "/end MEASUREMENT\n"
                    "\n"
                    "/begin MEASUREMENT NavDiag \"estimator-level diagnostics bitmask, NAVDIAG_* (Diagnostics.h) - a second word because Xcp_Data's diagStatus is full (32/32 bits); see docs/DIAGNOSTICS.md\"\n"
                    "  ULONG NO_COMPU_METHOD 0 0 0 4294967295\n"
                    "  ECU_ADDRESS 0x700305FC\n"
                    "/end MEASUREMENT\n"
                    "\n"
                    "/begin MEASUREMENT NavDiag_GnssUntrusted \"GNSS fix present but refused by the trust gate (hAcc above gnssHAccMax or debouncing): horizontal position is held, not updated - SWE1-FW-011\"\n"
                    "  ULONG NO_COMPU_METHOD 0 0 0 1\n"
                    "  ECU_ADDRESS 0x700305FC\n"
                    "  BIT_MASK 0x00000001\n"
                    "/end MEASUREMENT\n";
            }
            // QTextStream's destructor flushes it into the QIODevice, but the
            // QTemporaryFile itself must be CLOSED before a second QFile
            // (A2lModel::parseMeasurements opens its own handle on the same
            // path) can see the bytes on disk -- an open-but-unclosed
            // QTemporaryFile handle otherwise leaves the second reader
            // looking at a zero-length file on Windows.
            f.close();

            QString err;
            const QVector<A2lMeas> meas = A2lModel::parseMeasurements(f.fileName(), &err);
            // 5 rows: DiagStatus (plain word) + 2 bits, NavDiag (plain word) + 1 bit.
            allOk &= checkCase(report, meas.size() == 5,
                                "two-word fixture: A2lModel::parseMeasurements finds all 5 rows");

            const QVector<DiagWordGroup> g = buildDiagWordGroups(meas, fallback, "diagStatus");
            allOk &= checkCase(report, g.size() == 2,
                                "two-word fixture: exactly two word sections, both from the A2L");
            if (g.size() == 2) {
                allOk &= checkCase(report, g[0].addr == 0x70030024 && g[0].label == "DiagStatus"
                                                && g[0].rows.size() == 2,
                                    "two-word fixture: section 1 = DiagStatus, 2 bits");
                allOk &= checkCase(report, g[1].addr == 0x700305FC && g[1].label == "NavDiag"
                                                && g[1].rows.size() == 1 && g[1].rows[0].bit == 0,
                                    "two-word fixture: section 2 = NavDiag (own plain-word measurement), 1 bit");

                // Decode both words out of their own block -- the addresses
                // are > 256 bytes apart, so they land in two different
                // Xcp_Data/Xcp_Fusion blocks, exactly like the real firmware
                // layout (Xcp_Fusion is already in the DAQ list).
                QByteArray dataBlock(0x28, '\0');
                dataBlock[0x24] = char(0x03);   // both diagStatus bits set
                QByteArray fusionBlock(4, '\0');
                fusionBlock[0] = char(0x01);    // NavDiag bit 0 (GnssUntrusted) set

                double v0 = 0.0, v1 = 0.0;
                const bool ok0 = A2lModel::decodeFrom({0x70030000, 0x700305FC}, {dataBlock, fusionBlock}, g[0].wordMeas, &v0);
                const bool ok1 = A2lModel::decodeFrom({0x70030000, 0x700305FC}, {dataBlock, fusionBlock}, g[1].wordMeas, &v1);
                allOk &= checkCase(report, ok0 && quint32(v0) == 0x03u,
                                    "two-word fixture: DiagStatus word decodes from its own block (0x03)");
                allOk &= checkCase(report, ok1 && quint32(v1) == 0x01u,
                                    "two-word fixture: NavDiag word decodes from its own block (0x01), not DiagStatus's");
            }
        }
    }

    return allOk;
}
