use super::world::{Contact, World};

impl World {
    pub(super) fn print_debug_block(&self, contacts: &[Contact]) {
        println!("--- debug @ tick {}  epoch=0x{:016x} ---", self.tick, self.epoch_id);

        if self.debug.show_energy {
            let mut ke = 0.0f32;
            for i in 0..(self.bodies.len() as u32) {
                let im = self.bodies.inv_mass_of(i);
                if im > 0.0 {
                    let m = 1.0 / im;
                    let v = self.bodies.vel(i).lin;
                    ke += 0.5 * m * v.length_squared();
                }
            }
            println!("energy: KE_total = {:.6}", ke);
        }

        if self.debug.show_bodies {
            let mut lines = 0usize;
            for i in 0..(self.bodies.len() as u32) {
                let p = self.bodies.pose(i).pos;
                let v = self.bodies.vel(i).lin;
                println!(
                    "body {:3}  pos=({:+.3},{:+.3},{:+.3})  vel=({:+.3},{:+.3},{:+.3})",
                    i, p.x, p.y, p.z, v.x, v.y, v.z
                );
                lines += 1;
                if lines >= self.debug.max_lines { break; }
            }
        }

        if self.debug.show_contacts {
            if contacts.is_empty() {
                println!("contacts: (none)");
            } else {
                let mut shown = 0usize;
                for c in contacts.iter() {
                    println!(
                        "contact  cA={} cB={}  n=({:+.3},{:+.3},{:+.3})  depth={:.5}",
                        c.a_collider, c.b_collider, c.normal.x, c.normal.y, c.normal.z, c.depth
                    );
                    shown += 1;
                    if shown >= self.debug.max_lines { break; }
                }
            }
        }
    }
}
