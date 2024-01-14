class Measures():
    def __init__(self, q=[], qDot=[], qDotDot=[], x=[], xDot=[], xDotDot=[]) -> None:
        self.q = q
        self.qDot = qDot
        self.qDotDot = qDotDot
        self.x = x
        self.xDot = xDot
        self.xDotDot = xDotDot

class Ref(Measures):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

class Real(Measures):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
