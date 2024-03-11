///
/// @file ScanVisDirFlag.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#ifndef __SCAN_VIS_DIR_FLAG_HXX_F8204A77A34D__
#define __SCAN_VIS_DIR_FLAG_HXX_F8204A77A34D__

namespace avoid {

enum ScanVisDirFlag
{
    VisDirNone = 0,
    VisDirUp   = 1,
    VisDirDown = 2
};

typedef unsigned int ScanVisDirFlags;

}  // namespace avoid

#endif  // __SCAN_VIS_DIR_FLAG_HXX_F8204A77A34D__
