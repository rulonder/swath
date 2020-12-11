import("../pkg/index").catch(console.error).then((wasm)=>{
    const {Plane,Ray} = wasm;
    const t0 = performance.now();
    for (let index = 0; index < 10000; index++) {
        const plane = new Plane(1,0,0,0,1,1,0,0,1);
        plane.tangents(1)
    }
    const plane = new Plane(1,1,1,1,1,1,0,0,1);
    console.log(plane.tangents(1));
    const t1 = performance.now();
    console.log(`Call to doSomething took ${t1 - t0} milliseconds.`);
    const tb0 = performance.now();
    for (let index = 0; index < 10000; index++) {
        const ray = new Ray(1,1,0,0,1,1);
        ray.intersects(1)
    }
    const ray = new Ray(0,0,0,0,0,1);
    console.log(ray.intersects(1))
    const tb1 = performance.now();
    console.log(`Call to doSomething took ${tb1 - tb0} milliseconds.`);
});
