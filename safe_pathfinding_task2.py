import click
from typing import Optional
from events import log
from maps import Location, Map
from parsing import validate_location, validate_map
from task2 import *


def find_shortest_safe_path(start: Location, goal: Location,
                            terrain_map: Map, terrain_threshold: int,
                            success_map: Map, success_threshold: float) \
        -> tuple[Optional[int], Optional[float], Optional[list[Location]]]:
    """Finds the path with lowest total cost that also satisfies 
       the minimum success probability threshold (Task 2).
       Returns (cost,prob_success,list(locations)) when a path is found.
       Returns (None,None,None) if no path is found."""

    find_path = task2(start, goal, terrain_map, terrain_threshold, success_map, success_threshold)
    return find_path.astart_algorithm()

    # This is the entry point for your code for Task 2.
    # Please create additional functions and classes etc as needed 
    # to structure your implementation. 
    # Avoid implementing the entire algorithm in one long chunk.

    # raise NotImplementedError


@click.command(no_args_is_help=True)
@click.argument('start', required=True, callback=validate_location)
@click.argument('goal', required=True, callback=validate_location)
@click.argument("terrain_map", required=True, type=click.Path(exists=True), callback=validate_map)
@click.argument("terrain_threshold", required=True, type=click.IntRange(min=0, max=1000))
@click.argument("success_map", required=True, type=click.Path(exists=True), callback=validate_map)
@click.argument("success_threshold", required=True, type=click.FloatRange(min=0.0, max=1.0))
def main(start: Location, goal: Location,
         terrain_map: Map, success_map: Map,
         terrain_threshold: int, success_threshold: float) -> None:
    """Example usage:

        \b
        python safe_pathfinding_task2.py 3,2 0,3 resources/terrain01.txt 50 resources/enemy01.txt 1.0
    """
    path = find_shortest_safe_path(start, goal, terrain_map, terrain_threshold, success_map, success_threshold)
    if path[0] is not None and path[1] is not None and path[2] is not None:
        log(f"The path is {path[2]} with cost {path[0]} and success probability {path[1]}")
    else:
        log('No path found')


if __name__ == '__main__':
    main()
