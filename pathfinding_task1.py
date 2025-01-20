import click
from typing import Optional
from events import log
from maps import Location, Map
from parsing import validate_location, validate_map
from task1 import *


def find_shortest_path(start: Location, goal: Location,
                       terrain_map: Map, terrain_threshold: int) \
        -> tuple[Optional[int], Optional[list[Location]]]:
    """Finds the path with lowest total cost (Task 1)
       Returns (cost,list(locations)) when a path is found.
       Returns (None,None) if no path is found."""

    find_path = task1(start, goal, terrain_map, terrain_threshold)
    return find_path.astart_algorithm()
    # This is the entry point for your code for Task 1.
    # Please create additional functions and classes etc as needed 
    # to structure your implementation. 
    # Avoid implementing the entire algorithm in one long chunk.

    # raise NotImplementedError


@click.command(no_args_is_help=True)
@click.argument('start', required=True, callback=validate_location)
@click.argument('goal', required=True, callback=validate_location)
@click.argument("terrain_map", required=True, type=click.Path(exists=True), callback=validate_map)
@click.argument("terrain_threshold", required=True, type=click.IntRange(min=0, max=1000))
def main(start: Location, goal: Location, terrain_map: Map, terrain_threshold: int) -> None:
    """Example usage:

    \b
    python pathfinding_task1.py 3,2 0,3 resources/terrain01.txt 50
    """
    path = find_shortest_path(start, goal, terrain_map, terrain_threshold)
    if path[0] is not None and path[1] is not None:
        log(f"The path is {path[1]} with cost {path[0]}.")
    else:
        log('No path found')


if __name__ == '__main__':
    main()
