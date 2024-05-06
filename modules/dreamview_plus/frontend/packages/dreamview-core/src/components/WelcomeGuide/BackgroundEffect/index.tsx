import React, { useRef, useEffect } from 'react';
import * as THREE from 'three';
import { EffectComposer } from 'three-stdlib/postprocessing/EffectComposer';
import { RenderPass } from 'three-stdlib/postprocessing/RenderPass';

function BackgroundEffect() {
    const containerRef = useRef();

    useEffect(() => {
        const width = containerRef.current.clientWidth;
        const height = containerRef.current.clientHeight;

        const scene = new THREE.Scene();
        const camera = new THREE.PerspectiveCamera(90, width / height, 0.1, 1000);
        const renderer = new THREE.WebGLRenderer({ alpha: true });

        renderer.setSize(width, height);
        containerRef.current.appendChild(renderer.domElement);

        const geometry = new THREE.SphereGeometry(0.008, 32, 32);
        const material = new THREE.MeshBasicMaterial({ color: '#1E72E5' });

        const sphereGroup = new THREE.Group();
        for (let i = 0; i < 500; i += 1) {
            const sphere = new THREE.Mesh(geometry, material);
            sphere.position.set((Math.random() - 0.5) * 10, (Math.random() - 0.5) * 10, (Math.random() - 0.5) * 10);
            sphereGroup.add(sphere);
        }

        scene.add(sphereGroup);
        camera.position.z = 5;

        const composer = new EffectComposer(renderer);
        composer.addPass(new RenderPass(scene, camera));

        const animate = () => {
            requestAnimationFrame(animate);

            sphereGroup.rotation.x += 0.01;
            sphereGroup.rotation.y += 0.01;

            composer.render();
        };

        animate();

        const onWindowResize = () => {
            const width = containerRef.current.clientWidth;
            const height = containerRef.current.clientHeight;

            renderer.setSize(width, height);
            camera.aspect = width / height;
            camera.updateProjectionMatrix();
        };

        window.addEventListener('resize', onWindowResize);

        return () => {
            renderer.dispose();
            geometry.dispose();
            material.dispose();
            window.removeEventListener('resize', onWindowResize);
        };
    }, []);

    return <div ref={containerRef} style={{ width: '100vw', height: '100vh', position: 'absolute' }} />;
}

export default BackgroundEffect;
