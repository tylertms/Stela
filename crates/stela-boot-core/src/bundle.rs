//! A boot bundle's verified source description and the device tree the
//! engine publishes from it.
pub mod live_tree;
pub mod source;

/// Whether the firmware volume on GPT partition `partition` is searched for
/// the restore inventory. An install names its medium's partition
/// (`--install-source`), and then only that volume is: the same path on
/// another volume, such as a disk that was once a Stela stick and still has
/// its old files, is not the inventory. Without a named source every volume
/// is searched and the path has to be on exactly one.
pub fn inventory_volume(source: Option<[u8; 16]>, partition: Option<[u8; 16]>) -> bool {
    source.is_none_or(|source| partition == Some(source))
}

#[cfg(test)]
mod tests {
    use super::inventory_volume;

    #[test]
    fn a_named_install_source_is_the_only_volume_searched_for_the_inventory() {
        let (stick, old) = ([1; 16], [2; 16]);
        assert!(inventory_volume(Some(stick), Some(stick)));
        // A disk that still holds an earlier stick's files, or a volume
        // with no GPT identity, is passed over.
        assert!(!inventory_volume(Some(stick), Some(old)));
        assert!(!inventory_volume(Some(stick), None));
    }

    #[test]
    fn without_a_named_source_every_volume_is_searched() {
        assert!(inventory_volume(None, Some([2; 16])));
        assert!(inventory_volume(None, None));
    }
}
