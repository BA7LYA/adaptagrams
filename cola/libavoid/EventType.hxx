///
/// @file EventType.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

#ifndef __EVENT_TYPE_HXX_6FAFF33FBA51__
#define __EVENT_TYPE_HXX_6FAFF33FBA51__

namespace avoid {

// Note: Open must come first.
typedef enum
{
    Open      = 1,
    SegOpen   = 2,
    ConnPoint = 3,
    SegClose  = 4,
    Close     = 5
} EventType;

}  // namespace avoid

#endif  // __EVENT_TYPE_HXX_6FAFF33FBA51__
