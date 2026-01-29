use crate::mock::MockW25Q;

#[tokio::test]
async fn test_flash() {
    let mut memory = [0xFF; 8192]; // 8KB flash
    let mut flash = MockW25Q::new(&mut memory);

    // Erase sector
    flash.erase_sector(0).await.unwrap();
    
    // Write data
    let data = b"Hello, Flash!";
    flash.write(0, data).await.unwrap();
    
    // Read back
    let mut buf = [0u8; 13];
    flash.read(0, &mut buf).await.unwrap();
    
    assert_eq!(&buf, data);
}