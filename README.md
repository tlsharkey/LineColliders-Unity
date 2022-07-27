# Line Collider

Takes a Unity LineRenderer and creates colliders around the line. Colliders are *not* planar colliders like those generated by the `LineRenderer.BakeMesh()` function. Instead, colliders are a circular shape extruded along the line path.

## Importing into Project

Go to the Package Manager window, press the add button, and select "Add package from Git URL..."
![image](https://user-images.githubusercontent.com/33668799/116729324-e3b13a80-a99b-11eb-9009-ade4d52a5aee.png)

Paste in this link: `https://github.com/tlsharkey/LineCollider-Unity.git`
You may get an error saying import failed. **Paste in the link a second time.**
Unity will then import the package and display a series of errors in the Console. Unity will automatically fix all of these, you can safely clear them.
