import numpy as np
from itertools import tee, islice, chain, izip


class utils:

    #  https://stackoverflow.com/questions/1011938/python-previous-and-next-values-inside-a-loop
    @classmethod
    def iterate_with_previous_and_next(cls, some_iterable):
        prevs, items, nexts = tee(some_iterable, 3)
        prevs = chain([None], prevs)
        nexts = chain(islice(nexts, 1, None), [None])
        return izip(prevs, items, nexts)

    @classmethod
    def interpolate(cls, start, end, samples=5, decimals_to_round=3):
        data = []
        step_size = (end - start) / (samples - 1)
        intermediate = start
        for i in range(samples):
            data.append(intermediate)
            intermediate += step_size
        return np.round(data, decimals_to_round)

    @classmethod
    def normalize_vector(cls, vector):
        norm = np.linalg.norm(vector, ord=1)
        if norm == 0:
            norm = np.finfo(vector.dtype).eps
        return vector / norm