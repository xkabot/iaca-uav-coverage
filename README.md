# Current WEBOT setup

*All tests so far have been done with 4 drones

**For reference, each step is 32ms, so 15k steps is 8 minutes of simulated flight.


## v3.2
Travis - 04/16/2026

Ok so I was thinking about it and decided that a coverage radius of 30m is probably unrealistic, as to be able to see 30m in
each direction the onboard cameras probably can't see anything in enough detail for it to be useful. Thus, I have shrunk the radius
down to 10m, which I think is more appropriate.

Also, I just realized that nowhere I can see in the paper do they discuss the area coverage they got in their results (like percentage of total area covered),
which I feel is like... the **main** way to see performance of an *area coverage algorithm*... Anyways, back to my changes / improvements...

I think the over-zealous pick of 30m coverage radius in v3.1 was why we were able to get 99% coverage, especially with seemingly 
non-optimal flight paths. Here, we still get ~75%, which also might be an overstatement due to coverage radius still being too high,
but I think much better overall. Results for same test but 10m radius is below.

Also, remember that the paper did 100k timesteps, and I'm only doing 15k for sake of time, so any results I get would be a
fraction of the full runs. I plan to do the 100k timesteps once I am running the full area/grid size.

<img src="pics/v3/coverage2.png" alt="coverage graph" width="800"/>

<img src="pics/v3/paths2.png" alt="drone paths" width="800"/>

And just for shits and giggles here is 400m by 400m area, 300x300 grid size, 50k max steps (26:40min real time), still with 10m coverage radius
and 4 drones. Max speedup was ~6.25x. Overall coverage at end was ~65% which honestly isn't terrible maybe? idk...

<img src="pics/v3/paths3.png" alt="coverage graph" width="800"/>

<img src="pics/v3/coverage3.png" alt="drone paths" width="800"/>


## v3.1
Travis - 04/16/2026

Including the changes that Caleb did for v2, I created seperate python files to hold constants to more easily test changes.

Notice some missing things that the paper did that I didn't, that significantly increased performance (I forgot to add a supervisor step size).

I also added a boundary deterrent, so that if a drone is headed towards the edge of the map, seen in `iaca_drone.py` as the `boundary_force(...)` function. 
All this does is alter the direction of the drone slightly leaning it back towards the center of the area whenever it is near or outside the edge of the area. This 
helps the drone to turn around faster, as changing direction is one of the drones struggles. 

Before I go any further, note that v2 results are no longer comparable, as I changed the radius of what is considered "covered" when passed by a drone.
So dont try to compare coverage between the two too hard.

The paper did not give what their radius of coverage is, nor what sort of sensor was being used to decide coverage, so I
assumed that using a video camera from a relatively average altitude you could capture a decent area around yourself.

Now I am able to run on much larger areas in a reasonable amount of time. I did a 200m by 200m area with grid size of 200x200, 
using 15,000 steps, and a sensor radius of 30m (i think thats reasonable?), and it was able to do a beautiful 99% coverage. 
I redid it with another seed and it did nearly the same (96%). Not sure if this is just because 15k  steps is overkill for an area of this size, but 
I've attached the graphs below. As you can see, while the paths aren't pretty, the drones do a wonderful job of staying 
in the area, either curving to avoid going out of bounds, or clearly turning around just off map. I see this as about as good as you get for
staying in the area, but the pathing might leave a little to be desired.

Although I will say being able to coverage the 200m by 200m area in 8 minutes is pretty good I think.

Also to note, my max speedup is ~8x now. The biggest contributor is obviously the `SUPERVISOR_STEP_SIZE`, and the higher the better,
but I think 15 is a good spot for it, and the paper has similar.

<img src="pics/v3/coverage1.png" alt="coverage graph" width="800"/>

<img src="pics/v3/paths1.png" alt="drone paths" width="800"/>

## v2
Travis - 04/12/26 (later in the night after v1)

Ok so I made some changes to make my code more like the paper, while still being able to function somewhat correctly, but the 
code still needs to be improved. At current setup, max time multiplier is ~2.6x, for the tiny 30m by 30m area, 100x100 grid, sensor range of 20 cells (so like 6m), with 87.34% coverage.

Right now (and in v1 if i forgot to mention), instead of doing the full area (700m by 700m), I am testing in a ~30m by 30m area, 
and the drones are only flying at a max speed of 1m/s (edit: paper also has max speed of 1m/s). This is because the drones have a tendency to lose control and crash if they are 
flying too fast or try to rapidly change directions due to pheromones, and I haven't been able to find a good way to fix that yet.

For the size of the area, this is because if I do too large of an area while keeping the grid size the same (100x100) then each grid becomes 
too large and the drones end up overcorrecting to reach new squares quickly and crashing. Thus as far as I can tell we would need
to increase the grid size (which is probably why the paper sometimes refers to the grid as 500x500), but when I do that it runs 
insanely slow and I haven't had time to look into optimizing it yet. So thats for later.

But it kinda does an okay job compared to what the paper showed. Obviously its a terrible pathing but it is able to roughly
stay in the correct area and with 4 drones and 15k time-steps it is able to do about 80% coverage... 

Although I will say, if you look at the paths the drones take, some seem to almost follow their exact same path twice, which is odd.
Although now that I mention it, I dont know if the paper actually says what the pheromone radius is (how big of an area is considered covered when a drone flies over it),
so maybe that is the issue and I just have it set too small.

Also if you plan to run the plot script, I have no clue why the other graphs it makes look scuffed. i think i messed up the 
data collection or something.

Here is the current coverage graph and drone flight path for the 4 drones with 15k time-steps and a 30m by 30m area.


The blue drone actually looks pretty good, doing roughly rows back and forth at ujniform distance for part of it, and then you
get others like red which does a lap around the area then starts going randomly...

Also, while the lines do go straight off, in the sim the drones do turn around very quickly after leaving the operating area.

<img src="pics/v2/coverage.png" alt="coverage graph" width="800"/>

<img src="pics/v2/paths.png" alt="drone paths" width="800"/>


## v1
Travis - 04/12/26

Downloaded Webot and OSM area that the paper uses as their "map". Added 4 of the drones that the paper uses (Crazyflie quadcopters).
Tried making basic implementation to control them similar to AICA and was able to do it on a small scale (like a 30meter x 30meter area), 
and currently has performance issues if I try to scale it up (i already have some optimizations planned).

But the actual code is a kind of pain in the ass because you need to watch out for not telling the drones to move too far / too fast 
to avoid having them lose control and flip / crash. The drones came with a python file to do PID control on them, but still isn't perfect.

I tried to alter how my code works to follow the paper more accurately and the code actually performed worse, so for now this folder is the best i've done.


The python scripts can be found in `controllers/` with `iaca_drone/iaca_drone.py` to control individual drones and `iaca_supervisor/iaca_supervisor.py` the main supervisor script. 

I also generated a script to plot the results it finds.