## Description
This is source code for WebAssembly learning demo, where I created Physarum Simulation using chunks and entities objects. Both agents and pheromones are entities.  

To build:  
wasm-pack build --target web  

You will get pkg directory — this is your JS WASM package.  

Then you can create link or copy into webserver directory.  
And launch it with:  
./server  
