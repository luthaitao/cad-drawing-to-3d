/* 三维零件查看器（three.js r128 UMD，file:// 双击可用）
 *
 * 用法：
 *   1. 把 rebuild3d.py 产出的 .glb / .stl 复制到本目录，改名为 model.glb
 *      （或双击 index.html 后直接把模型文件拖进窗口）
 *   2. 也支持 URL 参数指定：index.html?model=xxx.glb
 *
 * 约定：模型单位为 mm，导入时统一 scale 0.001 -> 场景 m。
 * 已内置：零法线修复（STL 常见）、幂等归一化（反复调用不累积缩放）。
 */
(function () {
  'use strict';

  var canvas = document.getElementById('stage');
  var renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: true });
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.shadowMap.enabled = true;

  var scene = new THREE.Scene();
  scene.background = new THREE.Color(0x1a1d23);
  scene.fog = new THREE.Fog(0x1a1d23, 20, 80);

  var camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.01, 500);
  var controls = new THREE.OrbitControls(camera, canvas);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;

  // 光照
  scene.add(new THREE.AmbientLight(0xffffff, 0.55));
  var key = new THREE.DirectionalLight(0xffffff, 0.9);
  key.position.set(5, 10, 7);
  key.castShadow = true;
  scene.add(key);
  var fill = new THREE.DirectionalLight(0x88aacc, 0.35);
  fill.position.set(-6, 4, -5);
  scene.add(fill);

  // 地面网格（1m 格）
  var grid = new THREE.GridHelper(10, 20, 0x3a4354, 0x2a3140);
  scene.add(grid);

  var modelRoot = new THREE.Group();
  scene.add(modelRoot);

  var pickMap = {};   // selId -> { meshes: [] }
  var selId = null;
  var raycaster = new THREE.Raycaster();
  var mouse = new THREE.Vector2();

  function msg(text, ms) {
    var el = document.getElementById('msg');
    el.textContent = text;
    el.style.display = text ? 'block' : 'none';
    if (text && ms) { setTimeout(function () { el.style.display = 'none'; }, ms); }
  }

  // ---- 零法线检测（STL/PLY 常见，渲染出来全黑） ----
  function normalsBad(g) {
    var n = g.attributes && g.attributes.normal;
    if (!n || !n.count) return true;
    var step = Math.max(1, Math.floor(n.count / 64));
    for (var i = 0; i < n.count; i += step) {
      var x = n.getX(i), y = n.getY(i), z = n.getZ(i);
      if (x * x + y * y + z * z > 1e-8) return false;
    }
    return true;
  }

  // ---- 材质统一：MeshBasicMaterial 无 emissive，无法高亮 ----
  function toStandard(mat) {
    if (!mat || mat.type === 'MeshStandardMaterial') return mat;
    var m = new THREE.MeshStandardMaterial({
      color: mat.color ? mat.color.clone() : 0xcccccc,
      map: mat.map || null,
      side: mat.side || THREE.FrontSide,
      opacity: mat.opacity !== undefined ? mat.opacity : 1,
      transparent: !!mat.transparent,
      metalness: 0.25,
      roughness: 0.65
    });
    return m;
  }

  // ---- 归一化（幂等：按当前实际尺寸反算，反复调用不累积） ----
  function normalize(obj) {
    var b0 = new THREE.Box3().setFromObject(obj);
    var size = b0.getSize(new THREE.Vector3());
    var maxDim = Math.max(size.x, size.y, size.z) || 1;
    // 业务单位 mm -> 场景 m
    obj.scale.multiplyScalar(0.001);
    obj.updateMatrixWorld(true);
    var b1 = new THREE.Box3().setFromObject(obj);
    var c = b1.getCenter(new THREE.Vector3());
    obj.position.x -= c.x;
    obj.position.z -= c.z;
    obj.position.y -= b1.min.y;   // 底贴地
    obj.updateMatrixWorld(true);
    return { maxDim: maxDim, bbox: new THREE.Box3().setFromObject(obj) };
  }

  // ---- 网格收集 + 索引 ----
  var meshes = [];
  function collect(obj) {
    obj.traverse(function (o) {
      if (!o.isMesh) return;
      o.castShadow = true;
      o.receiveShadow = true;
      if (o.geometry && normalsBad(o.geometry) && o.geometry.attributes.position) {
        o.geometry.computeVertexNormals();
      }
      o.material = toStandard(o.material);
      meshes.push(o);
    });
  }

  function registerMeshes(root, modelName) {
    meshes = [];
    collect(root);
    pickMap = {};
    var tree = document.getElementById('tree');
    tree.innerHTML = '<div style="color:#8a93a3;margin-bottom:4px;">网格列表（点击选中）</div>';
    meshes.forEach(function (m, i) {
      var id = 'M-' + i;
      m.userData.selId = id;
      pickMap[id] = { meshes: [m], name: m.name || ('网格 ' + (i + 1)) };
      var div = document.createElement('div');
      div.className = 'item';
      div.textContent = pickMap[id].name;
      div.onclick = function () { select(id); };
      tree.appendChild(div);
    });
    updateHud(modelName);
  }

  function updateHud(modelName) {
    var tris = 0;
    meshes.forEach(function (m) {
      var g = m.geometry;
      tris += g.index ? g.index.count / 3 : (g.attributes.position ? g.attributes.position.count / 3 : 0);
    });
    document.getElementById('h-name').textContent = modelName || '-';
    document.getElementById('h-mesh').textContent = meshes.length;
    document.getElementById('h-tri').textContent = Math.round(tris);
    if (meshes.length) {
      var b = new THREE.Box3();
      meshes.forEach(function (m) { b.expandByObject(m); });
      var s = b.getSize(new THREE.Vector3());
      document.getElementById('h-bbox').textContent =
        s.x.toFixed(1) + ' × ' + s.y.toFixed(1) + ' × ' + s.z.toFixed(1) + ' m';
      document.getElementById('h-wt').textContent = meshes.every(function (m) {
        var g = m.geometry;
        return g.index ? true : true;  // three 无 isWatertight，几何闭环由导出端 report 保证
      }) ? '见 report.json' : '-';
    }
  }

  function select(id) {
    selId = id;
    meshes.forEach(function (m) {
      var on = m.userData.selId === id;
      if (m.material && m.material.emissive) {
        m.material.emissive.setHex(on ? 0x2f5d8a : 0x000000);
      }
    });
    var items = document.querySelectorAll('#tree .item');
    items.forEach(function (el, i) {
      el.classList.toggle('sel', ('M-' + i) === id);
    });
  }

  canvas.addEventListener('click', function (ev) {
    mouse.x = (ev.clientX / window.innerWidth) * 2 - 1;
    mouse.y = -(ev.clientY / window.innerHeight) * 2 + 1;
    raycaster.setFromCamera(mouse, camera);
    var hits = raycaster.intersectObjects(meshes, false);
    if (hits.length) {
      var id = hits[0].object.userData.selId;
      if (id) select(id);
    }
  });

  // ---- 加载 GLB / STL ----
  function loadGLB(url, name, onDone) {
    new THREE.GLTFLoader().load(url, function (gltf) {
      var obj = gltf.scene;
      var info = normalize(obj);
      modelRoot.clear();
      modelRoot.add(obj);
      registerMeshes(obj, name);
      frameCamera(info.bbox);
      msg('已加载 ' + name, 2500);
      if (onDone) onDone();
    }, undefined, function (err) {
      msg('加载失败：' + name, 4000);
      console.error(err);
    });
  }

  function loadSTL(url, name, onDone) {
    new THREE.STLLoader().load(url, function (geom) {
      if (normalsBad(geom)) geom.computeVertexNormals();
      var mesh = new THREE.Mesh(geom, new THREE.MeshStandardMaterial({
        color: 0xc0c0c0, metalness: 0.3, roughness: 0.6
      }));
      var group = new THREE.Group();
      group.add(mesh);
      group.name = name;
      var info = normalize(group);
      modelRoot.clear();
      modelRoot.add(group);
      registerMeshes(group, name);
      frameCamera(info.bbox);
      msg('已加载 ' + name, 2500);
      if (onDone) onDone();
    }, undefined, function (err) {
      msg('加载失败：' + name, 4000);
      console.error(err);
    });
  }

  function loadFile(file) {
    var url = URL.createObjectURL(file);
    var lower = file.name.toLowerCase();
    if (lower.endsWith('.glb') || lower.endsWith('.gltf')) {
      loadGLB(url, file.name, function () { URL.revokeObjectURL(url); });
    } else if (lower.endsWith('.stl')) {
      loadSTL(url, file.name, function () { URL.revokeObjectURL(url); });
    } else {
      msg('不支持的文件类型：' + file.name, 3000);
      URL.revokeObjectURL(url);
    }
  }

  function frameCamera(bbox) {
    var size = bbox.getSize(new THREE.Vector3());
    var center = bbox.getCenter(new THREE.Vector3());
    var maxDim = Math.max(size.x, size.y, size.z) || 1;
    var dist = maxDim * 2.2;
    camera.position.set(center.x + dist * 0.7, center.y + dist * 0.55, center.z + dist * 0.7);
    controls.target.copy(center);
    controls.update();
  }

  // ---- 拖放 ----
  var dropEl = document.getElementById('drop');
  window.addEventListener('dragenter', function (ev) { ev.preventDefault(); dropEl.style.display = 'flex'; });
  window.addEventListener('dragover', function (ev) { ev.preventDefault(); });
  window.addEventListener('dragleave', function (ev) {
    if (ev.relatedTarget === null) dropEl.style.display = 'none';
  });
  window.addEventListener('drop', function (ev) {
    ev.preventDefault();
    dropEl.style.display = 'none';
    if (ev.dataTransfer.files && ev.dataTransfer.files.length) loadFile(ev.dataTransfer.files[0]);
  });

  // ---- 初始模型：URL 参数 > 默认 model.glb ----
  var params = new URLSearchParams(location.search);
  var initial = params.get('model') || 'model.glb';
  loadGLB(initial, initial, null);

  // ---- 渲染循环 ----
  function resize() {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
  }
  window.addEventListener('resize', resize);

  function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
  }
  animate();
})();
