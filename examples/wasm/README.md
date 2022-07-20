# urdf-viz wasm example

[You can try this example online.](https://openrr.github.io/urdf-viz/?urdf=https://raw.githubusercontent.com/openrr/urdf-viz/main/sample.urdf)

## How to run

First, install [wasm-pack](https://rustwasm.github.io/wasm-pack).

Then, build and serve the example.

```sh
cd examples/wasm
npm install
npm run serve
```

Then, open http://localhost:8080 in a browser.

### Params

You can specify the same options as the urdf-viz argument as URL parameters (use `urdf=` parameter to specify the urdf file to be loaded). For example:

<http://localhost:8080/?urdf=https://raw.githubusercontent.com/openrr/urdf-viz/main/sample.urdf>

<img width="600" alt="sample-urdf-viz" src="https://user-images.githubusercontent.com/43724913/121161218-bbb0d300-c887-11eb-942d-989f20dcaa4d.png">
