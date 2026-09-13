// Headless, automated evidence for SWE1-GUI-001 (vv: unit): exercises
// attitudeSelfCheck() -- the same three-canonical-quaternion handedness
// check the Attitude tab runs at startup -- without a display, a GL context
// or a QApplication. The function under test only touches
// QMatrix4x4/QQuaternion/QVector3D/QVector4D value types, so a plain main()
// is enough; this deliberately does NOT instantiate AttitudeGLWidget itself
// (that needs a real GL context), only the free functions it exposes.
//
// Built and run by build.bat (local) and by
// .github/workflows/build.yml (CI, MSVC Release) -- see both for the exact
// invocation. Non-zero exit / any [FAIL] line means the handedness math
// regressed; that is the condition this test exists to catch before it ships
// silently in a Release build where Q_ASSERT is compiled out.
#include "attitudeglwidget.h"

#include <QString>
#include <cstdio>

int main()
{
    QString report;
    const bool ok = attitudeSelfCheck(&report);

    std::fputs(qUtf8Printable(report), stdout);
    std::fputs(ok ? "[attitude_selfcheck] PASS\n" : "[attitude_selfcheck] FAIL\n", stdout);

    return ok ? 0 : 1;
}
