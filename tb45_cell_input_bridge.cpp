#include "tb45_cell_input_bridge.h"

/*
 * Optional AppBlocks-style globals from main.cpp. Marking these as weak lets
 * this bridge compile/link even when main.cpp does not define them.
 */
extern std::string CELLAPN __attribute__((weak));
extern std::string CELLUN __attribute__((weak));
extern std::string CELLPW __attribute__((weak));
extern std::string CELLPIN __attribute__((weak));

extern "C" const char *tb45_main_get_cellapn(void)
{
    if (&CELLAPN == nullptr) {
        return nullptr;
    }
    return CELLAPN.c_str();
}

extern "C" const char *tb45_main_get_cellun(void)
{
    if (&CELLUN == nullptr) {
        return nullptr;
    }
    return CELLUN.c_str();
}

extern "C" const char *tb45_main_get_cellpw(void)
{
    if (&CELLPW == nullptr) {
        return nullptr;
    }
    return CELLPW.c_str();
}

extern "C" const char *tb45_main_get_cellpin(void)
{
    if (&CELLPIN == nullptr) {
        return nullptr;
    }
    return CELLPIN.c_str();
}
