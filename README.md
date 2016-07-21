# External Ballistics Library
C# port of the GNU Ballistics Library originally created by Derek Yates

original code can be found at:
https://sourceforge.net/projects/ballisticslib/

Folder "Ballistics" contains C# code - minimal changes to the original code

Folder "BallisticsLibrary" contains C++ code - refactored from procedural style to OOP. This is **very useful** if you need multiple projectiles - if you are making a game and/or you want to control the calculation step yourself.

Two bugs were fixed:

##1. bug in original code:

dv = retard(DragFunction,DragCoefficient,v+headwind);

"v" is in feet per second

"headwind" is in miles per hour

should be:

dv = retard(DragFunction,DragCoefficient,v+headwind*5280.0/3600.0);

##2. bug in original code:

ptr[10*n+5]=RadtoMOA(atan(ptr[10*n+4])); // Windage in MOA

should be:

ptr[10*n+5]=RadtoMOA(atan(ptr[10*n+4]/(12*x))); // Windage in MOA
