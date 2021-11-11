const width = 800, height = 600, padding = 20;

let data;
let agent_size = Math.sqrt(2)/4;
let connectedness = 2;
let graph, agent, t;

let graph_buffer;

const moves = {
    2: [[0,1], [1,0], [-1,0],  [0,-1]],
    3: [[0,1], [1,1], [1,0],  [1,-1],  [0,-1],  [-1,-1], [-1,0], [-1,1]],
    4: [[0,1], [1,1], [1,0],  [1,-1],  [0,-1],  [-1,-1], [-1,0], [-1,1],
        [1,2], [2,1], [2,-1], [1,-2], [-1,-2], [-2,-1], [-2,1],  [-1,2]],
    5: [[0,1],   [1,1],   [1,0],   [1,-1],  [0,-1],  [-1,-1], [-1,0], [-1,1],
        [1,2],   [2,1],   [2,-1],  [1,-2],  [-1,-2], [-2,-1], [-2,1], [-1,2],
        [1,3],   [2,3],   [3,2],   [3,1],   [3,-1],  [3,-2],  [2,-3], [1,-3],
        [-1,-3], [-2,-3], [-3,-2], [-3,-1], [-3,1],  [-3,2],  [-2,3], [-1,3]]
};

function readFiles() {
    let parser = new DOMParser();
    Object.keys(data).forEach(key => {
        data[key].ready = false;
        const input = data[key].input;
        if (input.files.length == 0) return;
        const reader = new FileReader();
        reader.onload = (e) => {
            data[key].xml = parser.parseFromString(e.target.result, 'text/xml');
            data[key].ready = true;
            parseFiles();
        };
        reader.readAsText(input.files[0]);
    });
}

function check(grid, i1, j1, i2, j2) {
    let src = createVector(i1, j1);
    let dst = createVector(i2, j2);
    let heading = p5.Vector.sub(dst, src);
    let perp = p5.Vector.rotate(heading, HALF_PI).normalize();
    let agent_side = p5.Vector.mult(perp, agent_size);

    let src1 = p5.Vector.add(src,agent_side);
    let src2 = p5.Vector.sub(src,agent_side);
    let dst1 = p5.Vector.add(dst,agent_side);
    let dst2 = p5.Vector.sub(dst,agent_side);

    let rectPos = createVector(-0.5,-0.5), rectSize = createVector(1, 1);

    for (let i=Math.min(i1,i2); i<=Math.max(i1,i2); i++) for (let j=Math.min(j1,j2); j<=Math.max(j1,j2); j++) if (grid[i][j] == 1) { // obstacle
        let pos = createVector(i, j).add(rectPos);
        if (collideLineRect(src1.x, src1.y, dst1.x, dst1.y, pos.x, pos.y, rectSize.x, rectSize.y)) {
            return false;
        }
        if (collideLineRect(src2.x, src2.y, dst2.x, dst2.y, pos.x, pos.y, rectSize.x, rectSize.y)) {
            return false;
        }
    }

    return true;
}

function parseGrid() {
    let width = Number(data.map.xml.getElementsByTagName('width')[0].childNodes[0].nodeValue);
    let height = Number(data.map.xml.getElementsByTagName('height')[0].childNodes[0].nodeValue);

    graph = {
        width: width-1,
        height: height-1
    };

    let grid = [];

    let rows = data.map.xml.getElementsByTagName('row');
    for (let i=0; i<height; i++) grid.push(rows[i].childNodes[0].nodeValue.split(/\s/).map(x => parseInt(x)));

    for (let i=0; i<height; i++) {
        for (let j=0; j<width; j++) {
            if (grid[i][j] == 0) {
                graph[i + height * j] = {
                    x: j,
                    y: graph.height-i,
                    neighbors: []
                };

                const m = moves[connectedness];
                for (let k=0; k<m.length; k++) {
                    if (i + m[k][0] < height && i + m[k][0] >= 0 && j + m[k][1] < width && j + m[k][1] >= 0 &&
                        grid[i+m[k][0]][j+m[k][1]] == 0 && check(grid, i, j, i+m[k][0], j+m[k][1])) {
                        graph[i + height * j].neighbors.push([j+m[k][1],graph.height-i-m[k][0]]);
                    }
                }
            }
        }
    }
}

function parseGraph() {
    graph = {
        width: 0,
        height: 0
    };

    let nodes = data.map.xml.getElementsByTagName('node');
    for (let i=0; i<nodes.length; i++) {
        let id = nodes[i].id;
        let pos = nodes[i].getElementsByTagName('data')[0].childNodes[0].nodeValue.split(',').map(x => parseFloat(x));
        graph[id] = {
            x: pos[1],
            y: pos[0],
            neighbors: []
        };
        graph.width = Math.max(graph.width, graph[id].x);
        graph.height = Math.max(graph.height, graph[id].y);
    }

    for (let i=0; i<nodes.length; i++) {
        let id = nodes[i].id;
        graph[id].y = graph.height - graph[id].y;
    }

    let edges = data.map.xml.getElementsByTagName('edge');
    for (let i=0; i<edges.length; i++) {
        let src = edges[i].getAttribute('source');
        let dst = edges[i].getAttribute('target');
        graph[src].neighbors.push([graph[dst].x, graph[dst].y]);
    }
}

function parseLog() {
    agent = {};

    let paths = data.log.xml.getElementsByTagName('path');
    for (let i=0; i<paths.length; i++) {
        let sections = paths[i].getElementsByTagName('section');
        agent[i] = [];
        let cumul = 0;
        for (let j=0; j<sections.length; j++) {
            let duration = parseFloat(sections[j].getAttribute('duration'));
            agent[i].push({
                x1: parseFloat(sections[j].getAttribute('start_j')),
                y1: graph.height-parseFloat(sections[j].getAttribute('start_i')),
                x2: parseFloat(sections[j].getAttribute('goal_j')),
                y2: graph.height-parseFloat(sections[j].getAttribute('goal_i')),
                t1: cumul,
                t2: cumul+duration
            });
            cumul += duration;
        }
    }

    t = 0;
}

function parseFiles() {
    if (!data.map.ready || !data.log.ready) return;

    if (data.cfg.ready) {
        if (data.cfg.xml.getElementsByTagName('agent_size').length > 0) {
            agent_size = Number(data.cfg.xml.getElementsByTagName('agent_size')[0].childNodes[0].nodeValue);
        }

        if (data.cfg.xml.getElementsByTagName('connectedness').length > 0) {
            connectedness = Number(data.cfg.xml.getElementsByTagName('connectedness')[0].childNodes[0].nodeValue);
        }
    }

    if (data.map.xml.getElementsByTagName('grid').length > 0) parseGrid();
    else parseGraph();

    createGraph();

    parseLog();
}

function createGraph() {
    graph_buffer = createGraphics(width, height);
    
    const realWidth = width - 2 * padding, realHeight = height - 2 * padding;
    const scaleX = realWidth / graph.width, scaleY = realHeight / graph.height;

    graph_buffer.translate(padding, padding);
    graph_buffer.scale(Math.min(scaleX, scaleY));

    graph_buffer.strokeWeight(agent_size/10);
    graph_buffer.stroke(150, 255);

    Object.keys(graph).forEach(key => {
        const node = graph[key];
        if (!node.neighbors) return;
        for (let i=0; i<node.neighbors.length; i++) {
            graph_buffer.line(node.x, node.y, node.neighbors[i][0], node.neighbors[i][1]);
        }
    });
}

function setup() {
    createCanvas(width, height);

    data = {
        cfg: {
            input: document.getElementsByName('cfg')[0]
        },
        map: {
            input: document.getElementsByName('map')[0]
        },
        log: {
            input: document.getElementsByName('log')[0]
        }
    };

    let button = document.getElementById('create');
    button.addEventListener('click', readFiles);
}

function draw() {
    clear();

    if (!graph) return;

    image(graph_buffer, 0, 0);

    const realWidth = width - 2 * padding, realHeight = height - 2 * padding;
    const scaleX = realWidth / graph.width, scaleY = realHeight / graph.height;

    translate(padding, padding);
    scale(Math.min(scaleX, scaleY));

    if (!agent) return;

    noStroke();

    Object.keys(agent).forEach(key => {
        const path = agent[key];
        for (let i=0; i<path.length; i++) {
            if (t < path[i].t2) {
                let x = path[i].x1 + (t - path[i].t1) * (path[i].x2 - path[i].x1) / (path[i].t2 - path[i].t1);
                let y = path[i].y1 + (t - path[i].t1) * (path[i].y2 - path[i].y1) / (path[i].t2 - path[i].t1);
                fill(0, 50, 100);
                ellipse(x, y, agent_size);
                return;
            } else if (i == path.length-1) {
                fill(0, 100, 50);
                ellipse(path[i].x2, path[i].y2, agent_size);
                return;
            }
        }
    });

    t += 0.04;
}
