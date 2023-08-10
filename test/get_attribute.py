# class Example(object):
#     bool143 = True
#     bool2 = True
#     blah = False
#     foo = True
#     foobar2000 = False

# example = Example()
# members = [attr for attr in dir(example) if not callable(getattr(example, attr)) and not attr.startswith("__")]
# print(members)

class Example(object):
    bool143 = True
    bool2 = True
    blah = False
    foo = True
    foobar2000 = False

    def as_list(self):
        ret = []
        for field in dir(Example):
           if getattr(self, field):
               ret.append(field)
        return ",".join(ret)
arr = [] 
example = Example()
arr = example.as_list()
