import os
from scipy import weave
from scipy.weave import converters

_weave_util_hpp = "\"" + os.path.dirname(os.path.realpath(__file__)) + "/WeaveUtils.hpp\""

def weave_cpp(code, vars, support = "", headers = []):
    weave.inline(
        code, 
        vars.keys(), 
        local_dict = vars,
        support_code = support, 
        type_converters = converters.blitz,
        compiler = 'gcc', 
        headers = headers + [_weave_util_hpp]
        )
