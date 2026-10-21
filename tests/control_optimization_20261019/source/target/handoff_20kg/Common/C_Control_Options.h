#ifndef CONTROL_OPT_OPTIONS_H
#define CONTROL_OPT_OPTIONS_H
#ifndef OPT_STILL_MS
#define OPT_STILL_MS 200
#endif
#ifndef OPT_CORR_PCT
#define OPT_CORR_PCT 100
#endif
#ifndef OPT_J_X10
#define OPT_J_X10 10
#endif
#ifndef OPT_EMF_PCT
#define OPT_EMF_PCT 0
#endif
#ifndef OPT_ACC_PCT
#define OPT_ACC_PCT 25
#endif
#ifndef OPT_VEL_PCT
#define OPT_VEL_PCT 70
#endif
#ifndef OPT_STREAM
#define OPT_STREAM 1
#endif
#ifndef OPT_BLEND_MS
#define OPT_BLEND_MS 50
#endif
_Static_assert(OPT_J_X10>=0 && OPT_J_X10<=40,"fixed inertia range");
_Static_assert(OPT_ACC_PCT>=10 && OPT_ACC_PCT<=100,"acceleration ratio range");
_Static_assert(OPT_BLEND_MS>=0 && OPT_BLEND_MS<=500,"handoff duration range");
#endif
