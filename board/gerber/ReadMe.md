# Export Gerber
The exported gerber files can be used to convert it into gcode for a mill
## Export settings

Open the board in KiCad and select:
File | Plot
### General
Plot format: Gerber
### Include Layer
Include the Layer ***B.Cu*** and ***Edge.Cuts***
[ ] Plot border and title block
[x] Plot footprint values
[x] Plot footprint reference
[ ] Force plotting of invisible values / refs
[x] Exclude PCB edge layer from other layers
[x] Exclude pads from silk screen
[ ] Do not tent vias
[x] Use auxilary axis as origin
Drill marks: None
Scaling: 1:1
Plot mode: Filled
Default line width: 0.1mm
[ ] Mirrored plot
[ ] Negated plot
### Gerber Options 
[ ] Use Protel filename extensions
[ ] Generate Geber job file
[ ] Substract soldermask from silkscreen
Coordinate format: 4.6, unit mm
[ ] Use extended X2 format
[ ] Include netlist attributes
### Doing
Click
* **Plot**
* **Generate Drill Files ...**
 * [x] PTH and NPTH in a single file
 * Map File Format: DXF
 * Drill Units: mm
 * Drill Origin: Auxilary axis
