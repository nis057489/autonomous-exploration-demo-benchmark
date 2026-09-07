// Emits voxels in REAL observation order: one pose-graph node (one scan) at a
// time, each voxel attributed to the node that first saw it. That ordering is
// what makes the churn measurement meaningful -- it is how a robot's map
// actually accumulates during SLAM.
package main

import (
	"bufio"
	"encoding/binary"
	"fmt"
	"io"
	"math"
	"os"
)

func quatRotate(w, qx, qy, qz, vx, vy, vz float64) (float64, float64, float64) {
	tx := 2 * (qy*vz - qz*vy)
	ty := 2 * (qz*vx - qx*vz)
	tz := 2 * (qx*vy - qy*vx)
	return vx + w*tx + (qy*tz - qz*ty), vy + w*ty + (qz*tx - qx*tz), vz + w*tz + (qx*ty - qy*tx)
}

func parse(raw []byte, nodeIDBytes, edgeWeightBytes int, res float64, out *bufio.Writer) error {
	offset, n := 0, len(raw)
	readU32 := func() (uint32, error) {
		if offset+4 > n { return 0, io.ErrUnexpectedEOF }
		v := binary.LittleEndian.Uint32(raw[offset : offset+4]); offset += 4; return v, nil
	}
	readF64 := func() (float64, error) {
		if offset+8 > n { return 0, io.ErrUnexpectedEOF }
		b := binary.LittleEndian.Uint64(raw[offset : offset+8]); offset += 8
		return math.Float64frombits(b), nil
	}
	nodes, err := readU32()
	if err != nil { return err }

	type key struct{ ix, iy, iz int64 }
	seen := make(map[key]struct{}, 1<<20)
	inv := 1.0 / res
	total := 0

	for ni := uint32(0); ni < nodes; ni++ {
		pc, err := readU32()
		if err != nil { return fmt.Errorf("node %d count: %w", ni, err) }
		lx := make([]float64, 0, pc); ly := make([]float64, 0, pc); lz := make([]float64, 0, pc)
		for pi := uint32(0); pi < pc; pi++ {
			dim, err := readU32()
			if err != nil { return err }
			if dim != 3 { return fmt.Errorf("node %d point %d dim %d", ni, pi, dim) }
			x, _ := readF64(); y, _ := readF64(); z, _ := readF64()
			lx = append(lx, x); ly = append(ly, y); lz = append(lz, z)
		}
		if offset+64 > n { return io.ErrUnexpectedEOF }
		offset += 4
		tx, _ := readF64(); ty, _ := readF64(); tz, _ := readF64()
		offset += 4
		qw, _ := readF64(); qx, _ := readF64(); qy, _ := readF64(); qz, _ := readF64()

		fresh := 0
		for i := range lx {
			wx, wy, wz := quatRotate(qw, qx, qy, qz, lx[i], ly[i], lz[i])
			k := key{int64(math.Floor((wx + tx) * inv)), int64(math.Floor((wy + ty) * inv)), int64(math.Floor((wz + tz) * inv))}
			if _, ok := seen[k]; ok { continue }
			seen[k] = struct{}{}
			fmt.Fprintf(out, "%d %d %d %d\n", ni, k.ix, k.iy, k.iz)
			fresh++; total++
		}
		_ = fresh
		idBytes := 4 + nodeIDBytes
		if offset+idBytes > n { return io.ErrUnexpectedEOF }
		offset += idBytes
	}
	fmt.Fprintf(os.Stderr, "nodes=%d voxels=%d\n", nodes, total)
	return nil
}

func main() {
	raw, err := os.ReadFile(os.Args[1])
	if err != nil { panic(err) }
	res := 0.2
	fmt.Sscanf(os.Args[3], "%f", &res)
	f, _ := os.Create(os.Args[2])
	defer f.Close()
	w := bufio.NewWriterSize(f, 1<<20)
	defer w.Flush()
	// try both graph layouts, same as graph2vxch
	for _, c := range [][2]int{{0, 4}, {4, 8}} {
		w2 := bufio.NewWriterSize(f, 1<<20)
		if err := parse(raw, c[0], c[1], res, w2); err == nil { w2.Flush(); return }
		f.Truncate(0); f.Seek(0, 0)
	}
	panic("could not parse graph")
}
