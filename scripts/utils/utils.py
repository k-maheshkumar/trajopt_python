import numpy as np
from itertools import tee, islice, chain, izip, cycle
import math


class Utils:

    #  https://stackoverflow.com/questions/1011938/python-previous-and-next-values-inside-a-loop
    @classmethod
    def iterate_with_previous_and_next(cls, some_iterable):
        prevs, items, nexts = tee(some_iterable, 3)
        prevs = chain([None], prevs)
        nexts = chain(islice(nexts, 1, None), [None])
        return izip(prevs, items, nexts)

    # https://stackoverflow.com/questions/323750/how-to-access-previous-next-element-while-for-looping
    @classmethod
    def neighborhood(cls, iterable):
        # iterator = iter(iterable)
        iterator = cycle(iterable)
        prev_item = None
        current_item = next(iterator)  # throws StopIteration if empty.
        for next_item in iterator:
            yield (prev_item, current_item, next_item)
            prev_item = current_item
            current_item = next_item
        yield (prev_item, current_item, None)

    @classmethod
    def interpolate(cls, start, end, samples=5, decimals_to_round=3):
        data = []
        step_size = (end - start) / (samples - 1)
        intermediate = start
        for i in range(samples):
            data.append(intermediate)
            intermediate += step_size
        return np.round(data, decimals_to_round)

    # https://stackoverflow.com/questions/21030391/how-to-normalize-an-array-in-numpy
    @classmethod
    def normalize_vector(cls, vector):
        norm = np.linalg.norm(vector, ord=1)
        if norm == 0:
            norm = np.finfo(vector.dtype).eps
        return vector / norm

    # https://stackoverflow.com/questions/4114921/vector-normalization
    @classmethod
    def normalize_vector1(cls, vector):
        vector_magnitude = math.sqrt(sum(vector[i] * vector[i] for i in range(len(vector))))
        # return [vector[i] / vector_magnitude for i in range(len(vector))]
        if vector_magnitude == 0:
            vector_magnitude = 0.00001
        for i in range(len(vector)):
            vector[i] =  vector[i] / vector_magnitude
        return vector