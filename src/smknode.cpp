#include <smknode.h>

//------------------------------------------------------------------------------------------------------------
int32_t SmkNode::getSetting(String name)
{
    int32_t ret = 0; //default
    for(auto s:settings)
    {
        if(s.name==name){
            ret = s.value;
            break;
        } 
    }
    return ret;
}

